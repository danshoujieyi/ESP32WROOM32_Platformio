// 条件编译：仅允许在ESP32平台编译，非ESP32平台会报错
#ifndef ESP32
  #error 此示例程序仅适用于ESP32平台
#endif
/*
 * LEDC Chan to Group/Channel/Timer Mapping
** ledc: 0  => Group: 0, Channel: 0, Timer: 0
** ledc: 1  => Group: 0, Channel: 1, Timer: 0
** ledc: 2  => Group: 0, Channel: 2, Timer: 1
** ledc: 3  => Group: 0, Channel: 3, Timer: 1
** ledc: 4  => Group: 0, Channel: 4, Timer: 2
** ledc: 5  => Group: 0, Channel: 5, Timer: 2
** ledc: 6  => Group: 0, Channel: 6, Timer: 3
** ledc: 7  => Group: 0, Channel: 7, Timer: 3
** ledc: 8  => Group: 1, Channel: 0, Timer: 0
** ledc: 9  => Group: 1, Channel: 1, Timer: 0
** ledc: 10 => Group: 1, Channel: 2, Timer: 1
** ledc: 11 => Group: 1, Channel: 3, Timer: 1
** ledc: 12 => Group: 1, Channel: 4, Timer: 2
** ledc: 13 => Group: 1, Channel: 5, Timer: 2
** ledc: 14 => Group: 1, Channel: 6, Timer: 3
** ledc: 15 => Group: 1, Channel: 7, Timer: 3
*/
#include <Arduino.h>
#include "Wire.h" // IIC库
#include <MPU6050_light.h>
#include <Esp32Tb6612Motor.h>
#include "Esp32PcntEncoder.h"
#include "PidController.h"
#include "Kinematics.h"
#include "Hcsr04.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// 引入MicroROS库
#include "micro_ros_platformio.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
// MicroROS工具
#include <micro_ros_utilities/string_utilities.h> // 字符串内存分配初始化管理工具 
// 引入Wifi库
#include <WiFi.h>
// 引入ROS2消息接口
#include <geometry_msgs/msg/twist.h> // 基础控制参数消息接口，cmd_vel消息
#include <nav_msgs/msg/odometry.h> // 里程计消息接口
#include <sensor_msgs/msg/laser_scan.h>// 引入 LaserScan 消息接口（标准ROS2传感器消息，用于发布雷达扫描数据）
#include <std_msgs/msg/header.h>
// 引入YDLIDAR X2/X2L型号激光雷达的驱动头文件（依赖之前的LDS_YDLIDAR_X2_X2L驱动库）
#include <LDS_YDLIDAR_X2_X2L.h>
// OLED显示屏
#include <fishbot_display.h>
/***********************************显示屏初始化参数****************************************/
//屏幕分辨率
// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// #define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// 创建显示屏实例
FishBotDisplay fishDisplay;
/***********************************雷达初始化参数******************************************/
const uint8_t LIDAR_PWM_PIN = 15;   // 雷达电机转速控制引脚（ESP32输出PWM信号，连接15号引脚）
const uint8_t LIDAR_TX_PIN = 17;    // 雷达的TX引脚（雷达发送数据，需连接ESP32的RX引脚，此处为17号）
const uint8_t LIDAR_RX_PIN = 16;    // 雷达的RX引脚（雷达接收数据，需连接ESP32的TX引脚，此处为16号）
#define LIDAR_PWM_FREQ    10000     // PWM信号频率：10千赫兹（电机控制常用频率，避免高频噪声）
#define LIDAR_PWM_BITS    11        // PWM分辨率：11位（最大占空比为2^11-1=2047，精度更高）
#define LIDAR_PWM_CHANNEL    8      // PWM通道：2（ESP32有16个PWM通道，需避开其他模块占用的通道）
#define LIDAR_SERIAL_RX_BUFFER_SIZE  2048  // 雷达串口接收缓冲区大小：1024字节（默认值）
HardwareSerial LidarSerial(1);      // ESP32硬件串口1（专门用于与雷达通信，区别于调试用的Serial串口）
LDS_YDLIDAR_X2_X2L lidar;           // YDLIDAR X2/X2L雷达驱动实例（所有雷达操作都通过此对象实现）
void lidar_init();
/***********************************IMU初始化参数******************************************/
#define SDA_MPU6050 18
#define SCL_MPU6050 19
MPU6050 mpu(Wire);
void imu_init();
/***********************************超声波初始化参数*****************************************/
#define TRIG 2
#define ECHO 12
Hcsr04 ultrasonic(TRIG, ECHO); // 创建一个名为ultrasonic的对象，用于测距
void ultrasonic_init();
/***********************************编码器初始化参数*****************************************/
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define LEFT_ENCODER_A 25
#define LEFT_ENCODER_B 26
#define RIGHT_ENCODER_A 32
#define RIGHT_ENCODER_B 33
#define WHEEL_DISTANCE_PER_PULSE_MM     0.1963495f   // 轮子对应的一个脉冲运动距离（毫米）
#define ENCODER_PULSES_PER_REVOLUTION  1024  // 轮子每圈脉冲数
Esp32PcntEncoder encoder[2]; // 创建一个名为encoder的对象，用于控制编码器
void encoder_init();
/***********************************电机PID初始化参数***************************************/
#define PID_KP          0.315f      // 比例系数（初始值，需调试）
#define PID_KI          0.1f      // 积分系数（初始值，需调试）
#define PID_KD          0.0001f      // 微分系数（初始值，需调试）
#define PID_INTEGRAL_LIM 20.0f    // 积分限幅（防止积分饱和）
#define PID_MAX_OUT     100.0f    // PID最大输出（对应电机PWM范围：0-100）
pid_obj_t *speed_pid[2] = {NULL, NULL};
void motor_pid_init();
/***********************************电机初始化参数******************************************/
#define LEFT_MOTOR_PWM 21
#define RIGHT_MOTOR_PWM 27
#define LEFT_MOTOR_AIN1 23
#define LEFT_MOTOR_AIN2 22
#define RIGHT_MOTOR_BIN1 13
#define RIGHT_MOTOR_BIN2 14
#define LEFT_MOTOR_PWM_DEAD_BAND 5.0f// 6.25f // 左电机死区（占空比%）
#define RIGHT_MOTOR_PWM_DEAD_BAND 5.0f// 6.25f // 右电机死区（占空比%）
Esp32Tb6612Motor motor; // 创建一个名为motor的对象，用于控制电机
void motor_init();
/***********************************底盘运动学解算初始化参数**********************************/
Kinematics kinematics; // 创建一个名为kinematics的对象，用于计算位置
#define WHEEL_DISTANCE 138.0f // 轮子间距（毫米）
void kinematics_init();

float target_linear_speed = 0.0; // 单位 毫米每秒
float target_angular_speed = 0.0; // 单位 弧度每秒
float out_left_speed = 0.0;       // 输出的左右轮速度，不是反馈的左右轮速度
float out_right_speed = 0.0;
/***********************************WIFI连接初始化参数*************************************/
// Agent服务器IP地址（热点、；路由器等等）
const char* agent_ip_str = "192.168.145.136";//192.168.145.136
const char* node_name = "fishbot_motion_control";// 节点名称（字符串形式，方便修改）
char wifi_ssid[] = "Liu";         // 代替直接传递 "Liu"
char wifi_password[] = "liujiajun"; // 代替直接传递 "liujiajun"
/***********************************MicroROS初始化参数*************************************/
bool connected_agent = false; // 时间同步标志位，其他线程想要使用MicroROS内置函数，必须等MicroROS线程与ROS2完成时间同步才允许运行，所以多线程必须检查该标志位
// 订阅者和计时器的数量（回调函数数量）
unsigned int ros_num_handles = 2;   // 需要严格对齐加入到执行器中的数量
// 1.引入MicroROS和Wifi对象
rcl_allocator_t allocator; // 内存分配器，动态分配内存
rclc_support_t support; // 用于存储时钟，内存分配器和上下文以及一些支持
rclc_executor_t executor; // 执行器，用于执行订阅者和发布者，管理订阅和定时器回调的执行
rcl_node_t node; // 节点，用于创建订阅者和发布者
// 2.创建节点订阅者和发布者，以及标准ROS2消息接口
// 创建订阅者，订阅ROS2下发的cmd_vel消息，并设置回调函数，存储下发的Twist消息
rcl_subscription_t sub_cmd_vel;
geometry_msgs__msg__Twist msg_cmd_vel;
void twist_callback(const void * msg_in);

// 创建发布者和定时器回调，存储发送的Odometry消息，创建定时器定时发布
rcl_publisher_t pub_odom;
nav_msgs__msg__Odometry msg_odom;
rcl_timer_t timer_odom;
void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time);

// 发布者全局变量（需在回调外定义，确保定时器能访问），填充ROS2 LaserScan消息
rcl_publisher_t pub_laser_scan;
sensor_msgs__msg__LaserScan msg_laser_scan;
/***********************************FreeRTOS多线程读写同步与保护初始化************************/
SemaphoreHandle_t scan_sem = NULL; // 新增
SemaphoreHandle_t scan_mutex = NULL;

void semaphore_init();
void mutex_init();
/***********************************多任务创建初始化参数*************************************/
void MicroROS_Task(void *args);
void Motor_Control_Task(void *args);
void Lidar_Task(void *args);
void IMU_Task(void *args);
void Task_Create_Init()  // FreeRTOS中时间片轮转第一次执行时，后创建的任务先运行（后创建任务先放入到队列头部）
{  
  xTaskCreatePinnedToCore(IMU_Task, "IMU_Task", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Motor_Control_Task, "Motor_Control_Task", 8192, NULL, 3, NULL, 1); // 创建雷达扫描线程任务
  xTaskCreatePinnedToCore(Lidar_Task, "Lidar_Task", 4096, NULL, 3, NULL, 1);
  // xTaskCreate(microros_task, "microros_task", 10240, NULL, 1, NULL); // 创建MicroROS单独线程任务
  xTaskCreatePinnedToCore(MicroROS_Task, "MicroROS_Task", 20480, NULL, 3, NULL, 0);
}
/***********************************必要消息接口初始化参数************************************/
// 全局双缓冲或共享缓冲（MicroROS_Task 在同一任务里发布，所以直接共享也安全，但写入时需要短锁）
static float ranges_buff[1024];      // 原始驱动写入缓冲（由 lidar callback 填充）
static float ranges_shared[1024];    // MicroROS_Task 从这里发布
static float intensities_buff[1024];  // 存放每个点的强度值
static float intensities_shared[1024];// MicroROS_Task 从这里发布
static size_t point = 0;
static size_t point_count =0;
static size_t latest_point_count = 0;

void MicroROS_pub_msg_init()
{
    // 初始化 Odometry
  nav_msgs__msg__Odometry__init(&msg_odom);
  msg_odom.header.frame_id = micro_ros_string_utilities_set(msg_odom.header.frame_id, "odom");
  msg_odom.child_frame_id = micro_ros_string_utilities_set(msg_odom.child_frame_id, "base_footprint");

  sensor_msgs__msg__LaserScan__init(&msg_laser_scan);  // 初始化消息结构体
  msg_laser_scan.header.frame_id = micro_ros_string_utilities_set( msg_laser_scan.header.frame_id, "laser_link");
  msg_laser_scan.range_min = 0.12f;
  msg_laser_scan.range_max = 8.0f;
  msg_laser_scan.angle_min = 0.0f;
  msg_laser_scan.angle_max = 2*M_PI; // 360度→2π弧度

  // 初始化 cmd_vel msg（executor 用到）
  geometry_msgs__msg__Twist__init(&msg_cmd_vel);
}
/***********************************板载LED初始化参数***************************************/
#define LED 2
void led_init();


void setup() {
  // 初始化顺序不能错
  Serial.begin(115200);
  semaphore_init();
  mutex_init();
  lidar_init();
  // led_init();
  // 初始化显示屏
  fishDisplay.init();
  Task_Create_Init();
}

void loop() 
{
  if (!connected_agent)
    return;

  delay(20);
}

void Motor_Control_Task(void *args)
{
  while (!connected_agent) {
    delay(100);
  }

  kinematics_init();
  encoder_init();
  motor_pid_init();
  motor_init();

  // 初始化时间戳变量，用于计算dt_ms
  uint64_t last_time = millis();  // 记录上一次循环的时间（毫秒）
  while (true)
  {
    // 1. 计算当前循环与上一次的时间间隔dt_ms
    uint64_t current_time = millis();
    uint16_t dt_ms = current_time - last_time;  // 时间差（毫秒）
    last_time = current_time;  // 更新上一次时间戳

    float left_speed_measure = encoder[LEFT_MOTOR].getSpeed();
    float left_target = pid_calculate(speed_pid[LEFT_MOTOR], left_speed_measure, out_left_speed);
    float right_speed_measure = encoder[RIGHT_MOTOR].getSpeed();
    float right_target = pid_calculate(speed_pid[RIGHT_MOTOR], right_speed_measure, out_right_speed);
    motor.updateMotorSpeed(LEFT_MOTOR, left_target);
    motor.updateMotorSpeed(RIGHT_MOTOR, right_target);

    // Serial.printf("left_speed_measure: %f, \n", left_speed_measure);
    // Serial.printf("right_speed_measure: %f, \n", right_speed_measure);
    // 4. 调用里程计更新函数（核心步骤）
    // 注意：确保kinematics是全局或任务内可见的Kinematics实例
    kinematics.update_odom(dt_ms, left_speed_measure, right_speed_measure);
    delay(10);
  }
}

void Lidar_Task(void *args)
{
  while (!connected_agent) {
    delay(100);
  }


  while (true)
  {
    // 雷达扫描
    lidar.loop();
    delay(10);
  }
}

void IMU_Task(void *args)
{
  while (!connected_agent) {
    delay(100);
  }

  ultrasonic_init();
  imu_init();

  while (true)
  {
    mpu.update();
    // ultrasonic.printDistance();
    // Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
    // Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
    // Serial.print("\tY: ");Serial.print(mpu.getAccY());
    // Serial.print("\tZ: ");Serial.println(mpu.getAccZ());
  
    // Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
    // Serial.print("\tY: ");Serial.print(mpu.getGyroY());
    // Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
  
    // Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
    // Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
    
    // Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
    // Serial.print("\tY: ");Serial.print(mpu.getAngleY());
    // Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
    // Serial.println(F("=====================================================\n"));
    delay(10);
  }
}

void MicroROS_Task(void *args) 
{
  // 1.设置传输协议并延迟一段时间等待设置的完成
  IPAddress agent_ip;
  agent_ip.fromString(agent_ip_str); // 设置agent的IP地址(路由器、热点等等的IP地址)(服务器IP)
  // 2.告诉ESP32连接的WIFI
  set_microros_wifi_transports(wifi_ssid, wifi_password, agent_ip, 8888); // 连接一个WIFI，往agent服务器发送数据，数据送到8888端口
  delay(2000);
  
  // 3.初始化内存分配器
  allocator = rcl_get_default_allocator();
  // 4.初始化支持
  rclc_support_init(&support, 0, NULL, &allocator); // 初始化支持
  // 5.创建节点
  rcl_ret_t ret = rclc_node_init_default(&node, node_name, "", &support); // 初始化节点

  MicroROS_pub_msg_init();

  // 添加一个cmd_vel订阅者,订阅者指针、节点名、消息类型、topic名
  rclc_subscription_init_best_effort(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");


  // 初始化发布者
  rclc_publisher_init_best_effort(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");
  // 初始化定时器
  rclc_timer_init_default(&timer_odom, &support, RCL_MS_TO_NS(50), odom_timer_callback); // 50ms周期的定时器

  
  // 初始化发布者，下面这个有自动重传机制和缓存机制，只要上位机没收到就一直传，但是rclc_publisher_init_best_effort是尽力而为的发送，有可能丢包
  rclc_publisher_init_default(&pub_laser_scan, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan");

  // 6.初始化执行器
  rclc_executor_init(&executor, &support.context, ros_num_handles, &allocator); // 初始化执行器
    // 订阅者添加到执行器中
  rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &twist_callback, ON_NEW_DATA);
    // 定时器添加到执行器中
  rclc_executor_add_timer(&executor, &timer_odom);
  while (true)
  {
    if (!rmw_uros_epoch_synchronized()) // 判断时间是否同步
    {
      rmw_uros_sync_session(1000); //  同步时间
      continue;
    }
    connected_agent = true;
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

       // 2) 如果有雷达数据准备好（由 lidar callback 填写 ranges_buff 并设置 scan_ready）
    if (xSemaphoreTake(scan_sem, pdMS_TO_TICKS(10)) == pdTRUE) {
      // 先拷贝共享缓冲到发布缓冲（由于在同一任务，理论上可直接使用共享缓冲，但仍用 mutex 保护写入阶段）
      if (xSemaphoreTake(scan_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        size_t pc = latest_point_count;
        if (pc > 0 && pc <= 1024) {
          // 填充其它参数（时间戳、angle_increment 等）
          int64_t time_stamp = rmw_uros_epoch_millis();
          msg_laser_scan.header.stamp.sec = static_cast<int32_t>(time_stamp / 1000);
          msg_laser_scan.header.stamp.nanosec = static_cast<uint32_t>((time_stamp % 1000) * 1000000);
          // 直接使用 ranges_shared (从 callback 已经拷贝过来的内容)，但为了保险，再次拷贝到本地发布缓冲
          msg_laser_scan.ranges.data = ranges_shared;
          msg_laser_scan.ranges.size = pc;
          msg_laser_scan.ranges.capacity = pc;
          msg_laser_scan.intensities.data = intensities_shared;
          msg_laser_scan.intensities.size = pc;
          msg_laser_scan.intensities.capacity = pc;

          if (pc > 0) {
            msg_laser_scan.angle_increment = (msg_laser_scan.angle_max - msg_laser_scan.angle_min) / (float)pc;
            float scan_freq = lidar.getCurrentScanFreqHz();
            if (scan_freq > 0.0f) {
              msg_laser_scan.scan_time = 1.0f / scan_freq;
              msg_laser_scan.time_increment = msg_laser_scan.scan_time / (float)pc;
            } else {
              msg_laser_scan.scan_time = 0.0f;
              msg_laser_scan.time_increment = 0.0f;
            }
          }

          rcl_ret_t pret = rcl_publish(&pub_laser_scan, &msg_laser_scan, NULL);
          if (pret != RCL_RET_OK) {
            Serial.printf("rcl_publish scan failed: %d\n", pret);
          }
        }
        xSemaphoreGive(scan_mutex);
        latest_point_count = 0;
      } else {
        // 若拿不到锁，跳过本帧（避免长时间阻塞）
        Serial.println("microros_task: scan_mutex busy, skip publish");
      }
    }

    delay(10);
  }
  // // 上下位机时间同步
  // while (!rmw_uros_epoch_synchronized())
  // {
  //   rmw_uros_sync_session(1000); // 一直调用上下位机同步函数，1000ms超时时间
  //   delay(10); // 延迟10ms,防止死循环
  // }
  // // 同步成功后，更新标志位
  // connected_agent = true;
  // // 7.循环执行器
  // rclc_executor_spin(&executor); // 循环执行器，直到程序结束
  // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); // 循环处理数据
}

void twist_callback(const void * msg_in)
{
  // target_linear_speed = 0;
  // target_angular_speed = 0;
  // 将收到的消息强制转换，一开始是void没有类型的数据指针
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
  target_linear_speed = msg->linear.x * 1000.0f; // 单位 米每秒（上位机） 转换为 毫米每秒（下位机）
  target_angular_speed = msg->angular.z; // 单位 弧度每秒（上下位机）
  kinematics.kinematics_inverse(target_linear_speed, target_angular_speed, &out_left_speed, &out_right_speed); // 初始位置为0，0，0
    // Serial.printf("out_left_speed: %f, \n", out_left_speed);
    // Serial.printf("out_right_speed: %f, \n", out_right_speed);
}

// 定时器回调函数
void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // 1.发布里程计时间戳，需要使用上下位机时间同步工具，做上下位机时间同步
  int64_t time_stamp = rmw_uros_epoch_millis(); // 获取当前时间戳
  msg_odom.header.stamp.sec = static_cast<int32_t>(time_stamp / 1000); // 毫秒转换秒
  msg_odom.header.stamp.nanosec = static_cast<uint32_t>((time_stamp % 1000) * 1000000); // 毫秒转换纳秒,先去除已经计算的秒的部分(求余)
  // 2.更新里程计消息
  odom_t odom = kinematics.get_odom();

  msg_odom.pose.pose.position.x = odom.x;
  msg_odom.pose.pose.position.y = odom.y;
  msg_odom.pose.pose.orientation.w = cos(odom.angle / 2.0f);
  msg_odom.pose.pose.orientation.z = sin(odom.angle / 2.0f);
  msg_odom.pose.pose.orientation.x = 0.0;
  msg_odom.pose.pose.orientation.y = 0.0;

  msg_odom.twist.twist.linear.x = odom.linear_speed;
  msg_odom.twist.twist.angular.z = odom.angular_speed;

  // 4.发布里程计消息
  if(rcl_publish(&pub_odom, &msg_odom, NULL) != RCL_RET_OK)
  {
    Serial.println("ERROR:Failed to publish Odom message");
  }
}


// 扫描点回调函数：雷达每输出一个扫描点数据，自动调用此函数
// 参数说明：
//   angle_deg：扫描点角度（单位：度）
//   distance_mm：扫描点距离（单位：毫米，<=0表示无效点）
//   quality：扫描点质量（数值越大，数据越可靠）
//   scan_completed：是否完成一圈扫描（true=完成一圈，false=未完成）
void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality,
  bool scan_completed) {
  if (scan_completed) {
    point_count = point;
    if (point_count == 0 || point_count > 1024) {
      point = 0;
      return;
    }

    // 将驱动缓冲（ranges_buff）拷贝到共享缓冲 (使用 mutex)
    if (xSemaphoreTake(scan_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      memcpy(ranges_shared, ranges_buff, point_count * sizeof(float));
      memcpy(intensities_shared, intensities_buff, point_count * sizeof(float));
      latest_point_count = point_count;
      xSemaphoreGive(scan_mutex);
      xSemaphoreGive(scan_sem);
    } else {
      // 若拿不到锁则放弃本帧（避免阻塞）
      Serial.println("lidar callback: scan_mutex busy, drop frame");
    }
    point = 0;
    point_count = 0;
    // 清理原驱动缓存
    memset(ranges_buff, 0, sizeof(ranges_buff));
    memset(intensities_buff, 0, sizeof(intensities_buff));

  } else {
    // 2. 【修正】有效距离判断：严格按手册0.12m~8m过滤（原仅判断distance_mm>0，漏过滤近距无效数据）
    float distance_m = distance_mm / 1000.0f; // 毫米→米（正确，保留）
    // if (distance_m > 0.0f && distance_m <= 0.12f)
    //   Serial.println(distance_m);
    // if (distance_m > 6.0f) 
    //   Serial.println(distance_m);
    bool is_valid_point = (distance_m >= msg_laser_scan.range_min) && (distance_m <= msg_laser_scan.range_max);
    // 3. 【修正】极坐标系方向：手册规定“顺时针为正”，需调整y轴符号（原计算会导致点云左右反转）
    // float angle_rad = angle_deg * M_PI / 180.0f; // 度→弧度（正确，保留）
    // 原逻辑：pt.y = distance_m * sin(angle_rad)（数学逆时针为正，与手册冲突）
    // 修正后：pt.y = -distance_m * sin(angle_rad)（顺时针为正，匹配手册坐标系）
    // 填充LaserScanData（仅保留有效点，无效点填NaN或0，符合ROS规范）
    if (point < 1024) {  // 只在数组容量范围内存储数据
        if (is_valid_point) {
            ranges_buff[point] = distance_m;
            intensities_buff[point] = quality;
        } else {
          // ⚠️ 无效点：slam_toolbox 不接受 NaN，改为 Inf
          ranges_buff[point] = INFINITY;      // 表示“无回波”或“超出测距范围”
          intensities_buff[point] = 0.0f;     // 无效点强度清零
        }
        point++;  // 仅在有效存储后递增索引
    } else {
        Serial.println("Warning: Exceeded maximum point count (1024), data discarded");
    }
  }
}

// 串口读取回调函数：驱动需要从雷达读数据时，自动调用此函数
int lidar_serial_read_callback() {
  // 从雷达通信串口（LidarSerial）读取1字节数据，返回读取结果（-1表示无数据）
   return LidarSerial.read();
}
// 原始数据包回调函数：雷达每输出一个原始数据包，自动调用此函数
// （当前为空实现，若需要解析原始数据，可在此添加逻辑）
void lidar_packet_callback(uint8_t * packet, uint16_t length, bool scan_completed) {
  return;
}
// 电机引脚控制回调函数：驱动需要控制电机引脚（使能/转速）时，自动调用此函数
// 参数说明：
//   value：控制值（可能是引脚方向、电平或PWM占空比）
//   lidar_pin：需要控制的雷达引脚（LDS_MOTOR_EN_PIN=使能引脚，LDS_MOTOR_PWM_PIN=PWM引脚）
void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {
  // 以下仅处理PWM引脚（你的雷达有此接口）
  // int pin = LIDAR_PWM_PIN; // 固定为PWM引脚
  // ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_BITS);
  // ledcAttachPin(pin, LIDAR_PWM_CHANNEL);
  // ledcWrite(LIDAR_PWM_CHANNEL, 100);
}
// 信息回调函数：雷达输出设备信息（如型号、固件版本）时，自动调用此函数
void lidar_info_callback(LDS::info_t code, String info) {
  Serial.print("激光雷达信息 [");
  Serial.print(lidar.infoCodeToString(code));  // 将信息码转为可读字符串（如“Model”“Firmware Version”）
  Serial.print("]：");
  Serial.println(info);  // 打印具体信息内容
}
// 错误回调函数：雷达发生错误（如超时、无效数据包）时，自动调用此函数
void lidar_error_callback(LDS::result_t code, String aux_info) {
  Serial.print("激光雷达错误 [");
  Serial.print(lidar.resultCodeToString(code));  // 将错误码转为可读字符串（如“Timeout error”“Invalid Packet error”）
  Serial.print("]：");
  Serial.println(aux_info);  // 打印辅助错误信息（如错误时的参数）
}
// 串口写入回调函数：驱动需要向雷达发指令时，自动调用此函数
size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length) {
  // 向雷达通信串口（LidarSerial）写入指定长度的数据，返回实际写入的字节数
  return LidarSerial.write(buffer, length);
}



void semaphore_init()
{
  scan_sem = xSemaphoreCreateBinary();
  if (scan_sem == NULL) {
    Serial.println("[ERROR] Failed to create scan_sem (binary semaphore)");
  }

}

void mutex_init()
{
  scan_mutex = xSemaphoreCreateMutex();
  if (scan_mutex == NULL) {
    Serial.println("[ERROR] Failed to create scan_mutex");
  }
}

void lidar_init()
{
  // 打印雷达型号（从驱动中获取，固定为"YDLIDAR X2/X2L"）
  Serial.print("激光雷达型号：");
  Serial.println(lidar.getModelName());
  // 调整雷达串口接收缓冲区大小（默认128字节硬件缓冲区+256字节软件缓冲区，此处改为1024字节避免数据溢出）
  Serial.print("激光雷达接收缓冲区大小：");
  Serial.print(LidarSerial.setRxBufferSize(LIDAR_SERIAL_RX_BUFFER_SIZE)); // 注意：此函数必须在串口.begin()之前调用
  Serial.println("字节");
  // 获取雷达默认串口波特率（X2/X2L型号固定为115200）
  uint32_t baud_rate = lidar.getSerialBaudRate();
  Serial.print("，串口波特率：");
  Serial.println(baud_rate);
  // 初始化雷达通信串口（波特率、数据格式、引脚映射）
  // SERIAL_8N1：8位数据位、1位停止位、无校验位（雷达默认通信格式）
  LidarSerial.begin(baud_rate, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  // 绑定驱动回调函数（驱动与ESP32硬件的“桥梁”，缺一不可）
  lidar.setScanPointCallback(lidar_scan_point_callback);    // 绑定“扫描点数据”回调（雷达输出扫描点时触发）
  lidar.setPacketCallback(lidar_packet_callback);          // 绑定“原始数据包”回调（雷达输出原始数据时触发，此处未使用）
  lidar.setSerialWriteCallback(lidar_serial_write_callback);// 绑定“串口写入”回调（驱动向雷达发数据时触发）
  lidar.setSerialReadCallback(lidar_serial_read_callback);  // 绑定“串口读取”回调（驱动从雷达读数据时触发）
  lidar.setMotorPinCallback(lidar_motor_pin_callback);      // 绑定“电机引脚控制”回调（驱动控制电机时触发）
  // 初始化雷达驱动（重置雷达内部状态，准备启动）
  lidar.init();
  // 启动雷达（使能电机、开始扫描），返回启动结果
  LDS::result_t result = lidar.start();
  // 打印启动结果（将结果码转为可读字符串，如“OK”或“Motor disabled error”）
  Serial.print("激光雷达启动结果：");
  Serial.println(lidar.resultCodeToString(result));
  // lidar.stop();  // 停止雷达（关闭电机、停止扫描）
  // 若启动失败（结果码<0），提示检查接线
  if (result < 0)
    Serial.println("提示：请检查激光雷达是否正确连接到ESP32？");
}

void imu_init()
{
  Wire.begin(SDA_MPU6050, SCL_MPU6050); // sda, scl
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  // 自动添加换行符语句
  Serial.println(status); 
  // 等待MPU6050初始化完成
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu6050偏移量清零
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
}

void ultrasonic_init()
{
  ultrasonic.begin();
}

void encoder_init()
{
  encoder[LEFT_MOTOR].init(LEFT_MOTOR,LEFT_ENCODER_A,LEFT_ENCODER_B);
  encoder[LEFT_MOTOR].setPeerPcntDistance(WHEEL_DISTANCE_PER_PULSE_MM);
  encoder[LEFT_MOTOR].setPulsesPerRevolution(ENCODER_PULSES_PER_REVOLUTION);
  encoder[RIGHT_MOTOR].init(RIGHT_MOTOR,RIGHT_ENCODER_A,RIGHT_ENCODER_B);
  encoder[RIGHT_MOTOR].setPeerPcntDistance(WHEEL_DISTANCE_PER_PULSE_MM);
  encoder[RIGHT_MOTOR].setPulsesPerRevolution(ENCODER_PULSES_PER_REVOLUTION);
}

void motor_pid_init()
{
  pid_config_t speed_pid_config = INIT_PID_CONFIG(PID_KP, PID_KI, PID_KD, PID_INTEGRAL_LIM, PID_MAX_OUT,
                                                    (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
  speed_pid[LEFT_MOTOR] = pid_register(&speed_pid_config);
  speed_pid[RIGHT_MOTOR] = pid_register(&speed_pid_config);
}

void motor_init()
{
  motor.attachMotor(LEFT_MOTOR, LEFT_MOTOR_AIN1, LEFT_MOTOR_AIN2, LEFT_MOTOR_PWM); // 将电机0连接到引脚33和引脚25
  motor.setMotorDeadBand(LEFT_MOTOR, LEFT_MOTOR_PWM_DEAD_BAND); // 设置电机0的死区为6.25
  motor.attachMotor(RIGHT_MOTOR, RIGHT_MOTOR_BIN1, RIGHT_MOTOR_BIN2, RIGHT_MOTOR_PWM); // 将电机1连接到引脚32和引脚26
  motor.setMotorDeadBand(RIGHT_MOTOR, RIGHT_MOTOR_PWM_DEAD_BAND); // 设置电机1的死区为6.25
}

void kinematics_init()
{
  kinematics.set_wheel_distance(WHEEL_DISTANCE); // 设置轮子间距
}

void led_init()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
}