#ifndef __INPUT_H
#define __INPUT_H

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

#include <sensor_msgs/msg/imu.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/takeoff_land.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "utils.h"
#include "PX4CtrlParam.h"

/**
 * @brief 遥控器数据类
 * 用于处理和存储来自遥控器的输入数据
 */
class RC_Data_t
{
public:
  rclcpp::Node::SharedPtr node_;    // ROS2节点指针
  double mode;                      // 当前飞行模式
  double gear;                      // 档位状态
  double reboot_cmd;               // 重启命令
  double last_mode;                // 上一次的飞行模式
  double last_gear;                // 上一次的档位状态
  double last_reboot_cmd;          // 上一次的重启命令
  bool have_init_last_mode{false};        // 是否初始化过上一次模式
  bool have_init_last_gear{false};        // 是否初始化过上一次档位
  bool have_init_last_reboot_cmd{false};  // 是否初始化过上一次重启命令
  double ch[4];                    // 遥控器四个通道的值

  mavros_msgs::msg::RCIn msg;      // 遥控器原始消息
  rclcpp::Time rcv_stamp;          // 接收时间戳

  bool is_command_mode;            // 是否处于命令模式
  bool enter_command_mode;         // 是否进入命令模式
  bool is_hover_mode;              // 是否处于悬停模式
  bool enter_hover_mode;           // 是否进入悬停模式
  bool toggle_reboot;              // 重启开关状态

  // 遥控器相关阈值常量
  static constexpr double GEAR_SHIFT_VALUE = 0.75;           // 档位切换阈值
  static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;   // API模式切换阈值
  static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;      // 重启阈值
  static constexpr double DEAD_ZONE = 0.25;                  // 死区值
  // static constexpr double TAKEOFF_LAND_THRESHOLD_VALUE = 0.75; // 起飞降落阈值

  RC_Data_t(const rclcpp::Node::SharedPtr& node);
  void check_validity();           // 检查数据有效性
  bool check_centered();           // 检查是否回中
  void feed(const mavros_msgs::msg::RCIn::SharedPtr pMsg);  // 数据输入
  bool is_received(const rclcpp::Time &now_time);           // 检查是否接收到数据
};

/**
 * @brief 里程计数据类
 * 用于处理和存储机体位置、速度和姿态信息
 */
class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  rclcpp::Node::SharedPtr node_;
  Eigen::Vector3d p;               // 位置
  Eigen::Vector3d v;               // 速度
  Eigen::Quaterniond q;            // 姿态四元数
  Eigen::Vector3d w;               // 角速度

  nav_msgs::msg::Odometry msg;     // 里程计原始消息
  rclcpp::Time rcv_stamp;          // 接收时间戳
  bool recv_new_msg;               // 是否接收到新消息

  Odom_Data_t(const rclcpp::Node::SharedPtr& node);
  void feed(const nav_msgs::msg::Odometry::SharedPtr pMsg);
};

/**
 * @brief IMU数据类
 * 用于处理和存储IMU传感器数据
 */
class Imu_Data_t
{
public:
  rclcpp::Node::SharedPtr node_;
  Eigen::Quaterniond q;            // 姿态四元数
  Eigen::Vector3d w;               // 角速度
  Eigen::Vector3d a;               // 加速度

  sensor_msgs::msg::Imu msg;       // IMU原始消息
  rclcpp::Time rcv_stamp;          // 接收时间戳

  Imu_Data_t(const rclcpp::Node::SharedPtr& node);
  void feed(const sensor_msgs::msg::Imu::SharedPtr pMsg);
};

/**
 * @brief 飞控状态数据类
 * 用于处理和存储PX4飞控状态信息
 */
class State_Data_t
{
public:
  rclcpp::Node::SharedPtr node_;
  mavros_msgs::msg::State current_state;           // 当前状态
  mavros_msgs::msg::State state_before_offboard;   // 进入offboard模式前的状态

  State_Data_t(const rclcpp::Node::SharedPtr& node);
  void feed(const mavros_msgs::msg::State::SharedPtr pMsg);
};

/**
 * @brief 扩展状态数据类
 * 用于处理和存储PX4扩展状态信息
 */
class ExtendedState_Data_t
{
public:
  rclcpp::Node::SharedPtr node_;
  mavros_msgs::msg::ExtendedState current_extended_state;  // 当前扩展状态

  ExtendedState_Data_t(const rclcpp::Node::SharedPtr& node);
  void feed(const mavros_msgs::msg::ExtendedState::SharedPtr pMsg);
};

/**
 * @brief 控制命令数据类
 * 用于处理和存储位置控制命令
 */
class Command_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  rclcpp::Node::SharedPtr node_;
  Eigen::Vector3d p;               // 位置命令
  Eigen::Vector3d v;               // 速度命令
  Eigen::Vector3d a;               // 加速度命令
  Eigen::Vector3d j;               // 加加速度命令
  double yaw;                      // 偏航角命令
  double yaw_rate;                 // 偏航角速度命令

  quadrotor_msgs::msg::PositionCommand msg;  // 位置命令原始消息
  rclcpp::Time rcv_stamp;                    // 接收时间戳

  Command_Data_t(const rclcpp::Node::SharedPtr& node);
  void feed(const quadrotor_msgs::msg::PositionCommand::SharedPtr pMsg);
};

/**
 * @brief 电池数据类
 * 用于处理和存储电池状态信息
 */
class Battery_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  rclcpp::Node::SharedPtr node_;
  double volt{0.0};                // 电压
  double percentage{0.0};          // 电量百分比

  sensor_msgs::msg::BatteryState msg;  // 电池状态原始消息
  rclcpp::Time rcv_stamp;              // 接收时间戳

  Battery_Data_t(const rclcpp::Node::SharedPtr& node);
  void feed(const sensor_msgs::msg::BatteryState::SharedPtr pMsg);
};

/**
 * @brief 起飞降落数据类
 * 用于处理和存储起飞降落命令
 */
class Takeoff_Land_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  rclcpp::Node::SharedPtr node_;
  bool triggered{false};                  // 是否触发
  uint8_t takeoff_land_cmd;               // 起飞降落命令

  quadrotor_msgs::msg::TakeoffLand msg;   // 起飞降落原始消息
  rclcpp::Time rcv_stamp;                 // 接收时间戳

  Takeoff_Land_Data_t(const rclcpp::Node::SharedPtr& node);
  void feed(const quadrotor_msgs::msg::TakeoffLand::SharedPtr pMsg);
};

#endif
