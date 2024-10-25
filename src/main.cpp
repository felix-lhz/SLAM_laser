#include <Arduino.h>
#include <WiFi.h>
#include <ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>
#include "MecanumDriver.h"
#include "rplidar.h"
#include "std_srvs/Empty.h"
#include "ros/service_server.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x) * M_PI / 180.)

using namespace rp::standalone::rplidar;

RPlidarDriver *drv = NULL;

// 创建麦克纳姆轮驱动器对象
MecanumDriver mecanum(9, 8, 12, 13, 11, 10, 46, 21);

// 创建激光雷达对象
// RPLidar lidar;

// 数据互斥锁
SemaphoreHandle_t mutex;

// WiFi网络名称和密码
const char *ssid = "evalley-1";
const char *password = "egu123456";

// ROS服务器的IP地址和端口
IPAddress server(192, 168, 1, 106);
const uint16_t serverPort = 11411;

// 创建ROS节点句柄
ros::NodeHandle nh;

// 定义发布的消息类型和话题
sensor_msgs::LaserScan scan_msg;
ros::Publisher scan_pub("scan", &scan_msg);

ros::Time start_scan_time;
ros::Time end_scan_time;
double scan_duration;
u_result op_result;
std::string frame_id;
bool inverted = false;
bool angle_compensate = true;
float max_distance = 8.0;
int angle_compensate_multiple =
    1; // it stand of angle compensate at per 1 degree
std::string scan_mode;

bool rplidar_init();
static float getAngle(const rplidar_response_measurement_node_hq_t &node);
bool start_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool stop_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool checkRPLIDARHealth(RPlidarDriver *drv);
bool getRPLIDARDeviceInfo(RPlidarDriver *drv);
void publish_scan(
    ros::Publisher *pub, rplidar_response_measurement_node_hq_t *nodes,
    size_t node_count, ros::Time start, double scan_time, bool inverted,
    float angle_min, float angle_max, float max_distance,
    std::string frame_id) ;
void cmdVelCallback(const std_msgs::Float64MultiArray &msg);
ros::Subscriber<std_msgs::Float64MultiArray> vel_sub("cmd_vel", &cmdVelCallback);

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200); // 打开与电脑调试的串口

    

    // mutex = xSemaphoreCreateMutex();
    mecanum.begin(); // 初始化麦克纳姆轮驱动器
    Serial.println("Mecanum driver initialized");

    WiFi.begin(ssid, password); // 连接到WiFi网络
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // // setup rplidar driver
    // drv = RPlidarDriver::CreateDriver();
    // // connect to selected serial port
    // drv->connect(&Serial2);
    while(!rplidar_init()) {
        Serial.println("Failed to initialize RPLIDAR");
        delay(100);
    }
    Serial.println("RPLIDAR initialized");

    nh.getHardware()->setConnection(server, serverPort);
    Serial.println("Connecting to ROS server");
    // 初始化ROS节点
    nh.initNode();
    Serial.println("ROS node initialized");
    nh.subscribe(vel_sub);
    Serial.println("Subscribed to cmd_vel topic");
    nh.advertise(scan_pub);
    Serial.println("Advertised scan topic");
}



void loop() {

    Serial.println("Entering loop");

    rplidar_response_measurement_node_hq_t nodes[360 * 8];
    size_t count = _countof(nodes);

    start_scan_time = nh.now();
    Serial.println("Starting scan");
    op_result = drv->grabScanDataHq(nodes, count);
    Serial.println("Scan data grabbed");
    end_scan_time = nh.now();
    scan_duration = (end_scan_time - start_scan_time).toSec();

    if (op_result == RESULT_OK) {
        op_result = drv->ascendScanData(nodes, count);
        float angle_min = DEG2RAD(0.0f);
        float angle_max = DEG2RAD(359.0f);
        if (op_result == RESULT_OK) {
            if (angle_compensate) {
                // const int angle_compensate_multiple = 1;
                const int angle_compensate_nodes_count =
                    360 * angle_compensate_multiple;
                int angle_compensate_offset = 0;
                rplidar_response_measurement_node_hq_t
                    angle_compensate_nodes[angle_compensate_nodes_count];
                memset(angle_compensate_nodes, 0,
                       angle_compensate_nodes_count *
                           sizeof(rplidar_response_measurement_node_hq_t));

                int i = 0, j = 0;
                for (; i < count; i++) {
                    if (nodes[i].dist_mm_q2 != 0) {
                        float angle = getAngle(nodes[i]);
                        int angle_value =
                            (int)(angle * angle_compensate_multiple);
                        if ((angle_value - angle_compensate_offset) < 0)
                            angle_compensate_offset = angle_value;
                        for (j = 0; j < angle_compensate_multiple; j++) {
                            angle_compensate_nodes[angle_value -
                                                   angle_compensate_offset +
                                                   j] = nodes[i];
                        }
                    }
                }

                publish_scan(&scan_pub, angle_compensate_nodes,
                             angle_compensate_nodes_count, start_scan_time,
                             scan_duration, inverted, angle_min, angle_max,
                             max_distance, frame_id);
            } else {
                int start_node = 0, end_node = 0;
                int i = 0;
                // find the first valid node and last valid node
                while (nodes[i++].dist_mm_q2 == 0)
                    ;
                start_node = i - 1;
                i = count - 1;
                while (nodes[i--].dist_mm_q2 == 0)
                    ;
                end_node = i + 1;

                angle_min = DEG2RAD(getAngle(nodes[start_node]));
                angle_max = DEG2RAD(getAngle(nodes[end_node]));

                publish_scan(&scan_pub, &nodes[start_node],
                             end_node - start_node + 1, start_scan_time,
                             scan_duration, inverted, angle_min, angle_max,
                             max_distance, frame_id);
            }
        } else if (op_result == RESULT_OPERATION_FAIL) {
            // All the data is invalid, just publish them
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);

            publish_scan(&scan_pub, nodes, count, start_scan_time,
                         scan_duration, inverted, angle_min, angle_max,
                         max_distance, frame_id);
        }
    }

    nh.spinOnce();
}

// 定义订阅的消息类型和话题
void cmdVelCallback(const std_msgs::Float64MultiArray &msg) {
    extern MecanumDriver mecanum; // Declare mecanum as an external variable
    // Handle the incoming velocity message
    // 确保消息数组中有足够的元素
    if (msg.data_length != 4) {
        Serial.println("Error: Invalid message size");
        return;
    } else {
        // 从消息数组中获取速度值
        mecanum.driveAllMotor(msg.data[0], msg.data[1], msg.data[2],
                              msg.data[3]);
    }
}

void publish_scan(ros::Publisher *pub,
                  rplidar_response_measurement_node_hq_t *nodes,
                  size_t node_count, ros::Time start, double scan_time,
                  bool inverted, float angle_min, float angle_max,
                  float max_distance, std::string frame_id) {
    static int scan_count = 0;
    sensor_msgs::LaserScan scan_msg;

    scan_msg.header.stamp = start;
    scan_msg.header.frame_id =
        frame_id.c_str(); // scan_msg.header.frame_id = frame_id;
    scan_count++;

    bool reversed = (angle_max > angle_min);
    if (reversed) {
        scan_msg.angle_min = M_PI - angle_max;
        scan_msg.angle_max = M_PI - angle_min;
    } else {
        scan_msg.angle_min = M_PI - angle_min;
        scan_msg.angle_max = M_PI - angle_max;
    }
    scan_msg.angle_increment =
        (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count - 1);

    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)(node_count - 1);
    scan_msg.range_min = 0.15;
    scan_msg.range_max = max_distance; // 8.0;

    scan_msg.intensities = new float[node_count];
    scan_msg.ranges = new float[node_count];

    bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
    if (!reverse_data) {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
            if (read_value == 0.0)
                scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
            else
                scan_msg.ranges[i] = read_value;
            scan_msg.intensities[i] = (float)(nodes[i].quality >> 2);
        }
    } else {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
            if (read_value == 0.0)
                scan_msg.ranges[node_count - 1 - i] =
                    std::numeric_limits<float>::infinity();
            else
                scan_msg.ranges[node_count - 1 - i] = read_value;
            scan_msg.intensities[node_count - 1 - i] =
                (float)(nodes[i].quality >> 2);
        }
    }

    pub->publish(&scan_msg);
}

bool getRPLIDARDeviceInfo(RPlidarDriver *drv) {
    u_result op_result;
    rplidar_response_device_info_t devinfo;

    op_result = drv->getDeviceInfo(devinfo);
    if (IS_FAIL(op_result)) {
        if (op_result == RESULT_OPERATION_TIMEOUT) {
            Serial.printf(
                "Error, operation time out. RESULT_OPERATION_TIMEOUT! \n");
        } else {
            Serial.printf("Error, unexpected error, code: %x \n", op_result);
        }
        return false;
    }

    // print out the device serial number, firmware and hardware version
    // number..
    printf("RPLIDAR S\n: ");
    for (int pos = 0; pos < 16; ++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\n");
    printf("Firmware Ver: %d.%02d", devinfo.firmware_version >> 8,
             devinfo.firmware_version & 0xFF);
    printf("Hardware Rev: %d", (int)devinfo.hardware_version);
    return true;
}

bool checkRPLIDARHealth(RPlidarDriver *drv) {
    u_result op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) {
        Serial.printf("RPLidar health status : %d \n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            Serial.printf(
                "Error, rplidar internal error detected. Please reboot "
                "the device to retry.\n");
            return false;
        } else {
            return true;
        }

    } else {
        Serial.printf("Error, cannot retrieve rplidar health code: %x \n",
                      op_result);
        return false;
    }
}

bool stop_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    if (!drv)
        return false;

    printf("Stop motor");
    drv->stop();
    drv->stopMotor();
    return true;
}

bool start_motor(std_srvs::Empty::Request &req,
                 std_srvs::Empty::Response &res) {
    if (!drv)
        return false;
    printf("Start motor");
    drv->startMotor();
    drv->startScan(0, 1);
    return true;
}

static float getAngle(const rplidar_response_measurement_node_hq_t &node) {
    return node.angle_z_q14 * 90.f / 16384.f;
}

bool rplidar_init() {
    // create the driver instance
    drv = RPlidarDriver::CreateDriver();
    if (!drv) {
        Serial.printf("Create Driver fail, exit \n");
        return false;
    }
    // make connection...
    if (IS_FAIL(drv->connect(&Serial2))) {
        Serial.printf(
            "Error, cannot bind to the specified serial port Serial2 \n");
        RPlidarDriver::DisposeDriver(drv);
        return false;
    }

    // get rplidar device info
    if (!getRPLIDARDeviceInfo(drv)) {
        return false;
    }

    // check health...
    if (!checkRPLIDARHealth(drv)) {
        RPlidarDriver::DisposeDriver(drv);
        return false;}
    
    drv->startMotor();

    RplidarScanMode current_scan_mode;

    if (scan_mode.empty()) {
        op_result = drv->startScan(false /* not force scan */,
                                   true /* use typical scan mode */, 0,
                                   &current_scan_mode);
    } else {
        std::vector<RplidarScanMode> allSupportedScanModes;
        op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

        if (IS_OK(op_result)) {
            _u16 selectedScanMode = _u16(-1);
            for (std::vector<RplidarScanMode>::iterator iter =
                     allSupportedScanModes.begin();
                 iter != allSupportedScanModes.end(); iter++) {
                if (iter->scan_mode == scan_mode) {
                    selectedScanMode = iter->id;
                    break;
                }
            }

            if (selectedScanMode == _u16(-1)) {
                Serial.printf(
                    "scan mode `%s' is not supported by lidar, supported "
                    "modes: \n",
                    scan_mode.c_str());
                for (std::vector<RplidarScanMode>::iterator iter =
                         allSupportedScanModes.begin();
                     iter != allSupportedScanModes.end(); iter++) {
                    Serial.printf(
                        "\t%s: max_distance: %.1f m, Point number: %.1fK \n",
                        iter->scan_mode, iter->max_distance,
                        (1000 / iter->us_per_sample));
                }
                op_result = RESULT_OPERATION_FAIL;
            } else {
                op_result = drv->startScanExpress(false /* not force scan */,
                                                  selectedScanMode, 0,
                                                  &current_scan_mode);
            }
        }
    }

    if (IS_OK(op_result)) {
        // default frequent is 10 hz (by motor pwm value),
        // current_scan_mode.us_per_sample is the number of scan point per us
        angle_compensate_multiple =
            (int)(1000 * 1000 / current_scan_mode.us_per_sample / 10.0 / 360.0);
        if (angle_compensate_multiple < 1)
            angle_compensate_multiple = 1;
        max_distance = current_scan_mode.max_distance;
        Serial.printf(
            "current scan mode: %s, max_distance: %.1f m, Point number: "
            "%.1fK , angle_compensate: %d \n",
            current_scan_mode.scan_mode, current_scan_mode.max_distance,
            (1000 / current_scan_mode.us_per_sample),
            angle_compensate_multiple);
    } else {
        Serial.printf("Can not start scan: %08x! \n", op_result);
        return false;
    }

    return true;
}