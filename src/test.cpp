#include "test.h"
#include "FlightLogger.h"

Test::Test(const std::shared_ptr<rclcpp::Node>& node) 
    : node_(node), counter_(0) {
}

// 回调函数只保存数据
// 接收并保存飞控状态信息
void Test::stateCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    state_msg_ = msg;
}

// 接收并保存扩展状态信息
void Test::extendedStateCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
    extended_state_msg_ = msg;
}

// 接收并保存里程计信息
void Test::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_msg_ = msg;
}

// 接收并保存位置指令信息
void Test::cmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
    cmd_msg_ = msg;
}

// 接收并保存IMU数据
void Test::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_msg_ = msg;
}

// 接收并保存遥控器数据
void Test::rcCallback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg) {
    rc_msg_ = msg;
}

// 接收并保存电池状态信息
void Test::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    battery_msg_ = msg;
}

// 接收并保存起飞降落命令
void Test::takeoffLandCallback(const quadrotor_msgs::msg::TakeoffLand::SharedPtr msg) {
    takeoff_land_msg_ = msg;
    
    // 详细记录起飞降落命令接收情况
    FLIGHT_LOG_INFO(MISSION, "🎯 [TEST模式] 起飞降落命令接收 - 命令类型: %d", msg->takeoff_land_cmd);
    
    // 根据命令类型输出详细信息
    switch(msg->takeoff_land_cmd) {
        case 1:
            FLIGHT_LOG_INFO(MISSION, "🚀 [TEST模式] 起飞命令已接收 - 准备起飞");
            FLIGHT_LOG_INFO(MISSION, "📋 [TEST模式] 起飞命令详情 - 命令类型: %d", 
                       msg->takeoff_land_cmd);
            break;
        case 2:
            FLIGHT_LOG_INFO(MISSION, "🛬 [TEST模式] 降落命令已接收 - 准备降落");
            FLIGHT_LOG_INFO(MISSION, "📋 [TEST模式] 降落命令详情 - 命令类型: %d", 
                       msg->takeoff_land_cmd);
            break;
        default:
            FLIGHT_LOG_WARN(MISSION, "⚠️ [TEST模式] 未知的起飞降落命令: %d", msg->takeoff_land_cmd);
            break;
    }
}

// void Test::takeoffLandCallback(const std_msgs::msg::Bool::SharedPtr msg) {
//     takeoff_land_msg_ = msg;
//     // 起飞降落信息
//     FLIGHT_LOG_INFO(MISSION, "Takeoff/Land - Takeoff: %d", 
//                 takeoff_land_msg_->data);
// }


// 显示函数
void Test::displayMessages() {
    counter_++;
    // 100Hz -> 1Hz 需要每100次循环显示一次
    if(counter_ >= 100) {
        counter_ = 0;

        FLIGHT_LOG_INFO(MISSION, "---------------------------------------");
        FLIGHT_LOG_INFO(MISSION, "--------------information--------------");
        
        // 状态信息
        if(state_msg_) {
            FLIGHT_LOG_INFO(MISSION, "State - USB Connected: %d, Armed: %d, Nav State: %d", 
                        state_msg_->usb_connected, state_msg_->arming_state, state_msg_->nav_state);
        } else {
            FLIGHT_LOG_INFO(MISSION, "No State message received");
        }

        if(extended_state_msg_) {
            FLIGHT_LOG_INFO(MISSION, "Extended State - Landed: %d", 
                        extended_state_msg_->landed);
        } else {
            FLIGHT_LOG_INFO(MISSION, "No Extended State message received");
        }

        // 位置信息ok
        if(odom_msg_) {
            FLIGHT_LOG_INFO(MISSION, "Odometry - Position: [%f, %f, %f]", 
                        odom_msg_->pose.pose.position.x, 
                        odom_msg_->pose.pose.position.y, 
                        odom_msg_->pose.pose.position.z);
        } else {
            FLIGHT_LOG_INFO(MISSION, "No Odometry message received");
        }

        // 姿态信息   
        if(imu_msg_) {
            FLIGHT_LOG_INFO(MISSION, "IMU - Angular velocity: [%f, %f, %f]", 
                        imu_msg_->angular_velocity.x, 
                        imu_msg_->angular_velocity.y, 
                        imu_msg_->angular_velocity.z);
        } else {
            FLIGHT_LOG_INFO(MISSION, "No IMU message received");
        }

        // 位置指令信息
        if(cmd_msg_) {
            FLIGHT_LOG_INFO(MISSION, "Position Command - Position: [%f, %f, %f]", 
                        cmd_msg_->position.x, cmd_msg_->position.y, cmd_msg_->position.z);
        } else {
            FLIGHT_LOG_INFO(MISSION, "No Position Command message received");
        }

        // 遥控器信息
        if(rc_msg_) {
            FLIGHT_LOG_INFO(MISSION, "Manual Control - Roll: %f, Pitch: %f, Throttle: %f, Yaw: %f, Aux1-4: %f, %f, %f, %f",
                rc_msg_->roll, rc_msg_->pitch, rc_msg_->throttle, rc_msg_->yaw,
                rc_msg_->aux1, rc_msg_->aux2, rc_msg_->aux3, rc_msg_->aux4);
        } else {
            FLIGHT_LOG_INFO(MISSION, "No Manual Control message received");
        }

        // 电池信息
        if(battery_msg_) {
            FLIGHT_LOG_INFO(MISSION, "Battery - Voltage: %f V, Percentage: %f%%", 
                        battery_msg_->voltage, battery_msg_->percentage);
        } else {
            FLIGHT_LOG_INFO(MISSION, "No Battery message received");
        }

        // 起飞降落信息
        if(takeoff_land_msg_) {
            FLIGHT_LOG_INFO(MISSION, "Takeoff/Land - Command: %d", 
                        takeoff_land_msg_->takeoff_land_cmd);
        } else {
            FLIGHT_LOG_INFO(MISSION, "No Takeoff/Land message received");
        }
    }
}
