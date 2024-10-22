#include <Arduino.h>
#include <RPLIDAR.h>
#include <WiFi.h>
#include <ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>
#include "MecanumDriver.h"

// 创建麦克纳姆轮驱动器对象
MecanumDriver mecanum(9, 8, 12, 13, 11, 10, 46, 21);

// 创建激光雷达对象
RPLidar lidar;

// 数据互斥锁
SemaphoreHandle_t mutex;

// WiFi网络名称和密码
const char *ssid = "evalley-1";
const char *password = "egu123456";

// ROS服务器的IP地址和端口
IPAddress server(192, 168, 1, 108);
const uint16_t serverPort = 11411;

// 创建ROS节点句柄
ros::NodeHandle nh;

// 定义发布的消息类型和话题
sensor_msgs::LaserScan scan_msg;
ros::Publisher scan_pub("scan", &scan_msg);

// 定义订阅的消息类型和话题
void cmdVelCallback(const std_msgs::Float64MultiArray &msg) {
    extern MecanumDriver mecanum; // Declare mecanum as an external variable
    // Handle the incoming velocity message
    // 确保消息数组中有足够的元素
    if (msg.data_length != 4) {
        Serial.println("Error: Invalid message size");
        return;
    }
    else {
        // 从消息数组中获取速度值
        mecanum.driveAllMotor(msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
    }
}

ros::Subscriber<std_msgs::Float64MultiArray> vel_sub("cmd_vel", &cmdVelCallback);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // 打开与电脑调试的串口
  lidar.begin(Serial2); // 将激光雷达连接到串口2（19、20引脚）
  mutex = xSemaphoreCreateMutex();
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

  // 设置ROS服务器的IP地址和端口
  nh.getHardware()->setConnection(server, serverPort);

  // 初始化ROS节点
  nh.initNode();
  nh.subscribe(vel_sub);
  nh.advertise(scan_pub);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (nh.connected()) {
      Serial.println("Connected");
  } else {
      Serial.println("Not Connected");
  }
  nh.spinOnce();

  delay(50); // 控制激光雷达数据采集频率
}
