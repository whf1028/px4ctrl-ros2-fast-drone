#include "px4ctrl_node.h"
#include "FlightLogger.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cstdlib>

#define TEST_OPEN 0 // 0: 不测试，1: 测试

// 信号处理函数，用于优雅地处理Ctrl+C等终止信号
void mySigintHandler(int /* sig */)
{
    FLIGHT_LOG_INFO(PX4CTRL, "[px4ctrl_node] exit..."); // 打印退出信息
    rclcpp::shutdown();               // 关闭ROS 2节点
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*产生一个px4ctrl的节点*/
    auto node = std::make_shared<rclcpp::Node>("px4ctrl_node");
    // 设置日志级别为DEBUG
    (void)rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    
    // 设置全局日志级别为DEBUG
    (void)rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
    
    // 设置所有ROS2日志级别为DEBUG
    (void)rcutils_logging_set_logger_level("rclcpp", RCUTILS_LOG_SEVERITY_DEBUG);
    (void)rcutils_logging_set_logger_level("rcl", RCUTILS_LOG_SEVERITY_DEBUG);
    (void)rcutils_logging_set_logger_level("rcutils", RCUTILS_LOG_SEVERITY_DEBUG);
    
    // 注册信号处理函数
    signal(SIGINT, mySigintHandler);
    
    // 自动配置日志记录（如果没有环境变量）
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto session_id = "session_" + std::to_string(time_t);
    
    // 设置默认日志目录
    if (!std::getenv("FLIGHT_LOG_DIR")) {
        setenv("FLIGHT_LOG_DIR", "/home/fsuav/fast-drone-250-humble-6.1/analyze_rosbag", 0);
    }
    if (!std::getenv("FLIGHT_LOG_SESSION_ID")) {
        setenv("FLIGHT_LOG_SESSION_ID", session_id.c_str(), 0);
    }
    if (!std::getenv("FLIGHT_LOG_ENABLED")) {
        setenv("FLIGHT_LOG_ENABLED", "true", 0);
    }
    if (!std::getenv("FLIGHT_LOG_LEVEL")) {
        setenv("FLIGHT_LOG_LEVEL", "DEBUG", 0);
    }
    
    FLIGHT_LOG_INFO(SYSTEM, "FlightLogger: 自动配置日志记录");
    FLIGHT_LOG_INFO(SYSTEM, "日志目录: %s", std::getenv("FLIGHT_LOG_DIR"));
    FLIGHT_LOG_INFO(SYSTEM, "会话ID: %s", std::getenv("FLIGHT_LOG_SESSION_ID"));
    
    // 初始化FlightLogger
    auto& logger = px4ctrl::FlightLogger::getInstance();
    if (logger.initialize(std::getenv("FLIGHT_LOG_DIR"), std::getenv("FLIGHT_LOG_SESSION_ID"))) {
        FLIGHT_LOG_INFO(SYSTEM, "FlightLogger: 日志目录创建成功");
        FLIGHT_LOG_INFO(PX4CTRL, "PX4Ctrl节点启动，日志系统初始化完成");
        FLIGHT_LOG_DEBUG(SYSTEM, "系统环境配置完成 - 日志目录: %s, 会话ID: %s", 
                        std::getenv("FLIGHT_LOG_DIR"), std::getenv("FLIGHT_LOG_SESSION_ID"));
    } else {
        FLIGHT_LOG_WARN(SYSTEM, "FlightLogger: 日志系统初始化失败");
        FLIGHT_LOG_ERROR(SYSTEM, "日志系统初始化失败，将使用控制台输出");
    }
    
    rclcpp::sleep_for(std::chrono::seconds(1));  // 等待1秒，确保其他节点已经启动

    // 加载参数
    FLIGHT_LOG_INFO(PX4CTRL, "开始加载参数配置...");
    Parameter_t param;
    param.config_from_ros_handle(node);
    FLIGHT_LOG_INFO(PX4CTRL, "参数配置加载完成");
    FLIGHT_LOG_DEBUG(SYSTEM, "关键参数: 控制频率=%.1fHz, 质量=%.2fkg, 重力=%.2fm/s²", 
                    param.ctrl_freq_max, param.mass, param.gra);

    // 初始化控制器和状态机
    FLIGHT_LOG_INFO(CONTROLLER, "开始初始化LinearControl控制器...");
    LinearControl controller(param, node);  // 创建线性控制器实例
    FLIGHT_LOG_INFO(CONTROLLER, "LinearControl控制器初始化完成");
    
    FLIGHT_LOG_INFO(PX4CTRL, "开始初始化PX4CtrlFSM状态机...");
    PX4CtrlFSM fsm(param, controller, node);  // 创建状态机实例
    FLIGHT_LOG_INFO(PX4CTRL, "PX4CtrlFSM状态机初始化完成");
    FLIGHT_LOG_DEBUG(FLIGHT_PHASE, "初始状态: MANUAL_CTRL");
    #if TEST_OPEN
        // 创建测试类实例
        FLIGHT_LOG_INFO(ROS2, "创建测试类实例...");
        Test test(node);
        FLIGHT_LOG_INFO(ROS2, "测试类实例创建完成");

        // 1. 订阅飞控状态信息
        FLIGHT_LOG_INFO(SENSOR, "创建飞控状态订阅器: /fmu/out/vehicle_status_v1");
        auto state_sub = node->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", rclcpp::QoS(10).best_effort().durability_volatile(),
            std::bind(&Test::stateCallback, &test, std::placeholders::_1));
        FLIGHT_LOG_DEBUG(SENSOR, "飞控状态订阅器创建完成，QoS: best_effort");

        // 2. 订阅扩展状态信息
        FLIGHT_LOG_INFO(SENSOR, "创建扩展状态订阅器: /fmu/out/vehicle_land_detected");
        auto extended_state_sub = node->create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected", rclcpp::QoS(10).best_effort().durability_volatile(),
            std::bind(&Test::extendedStateCallback, &test, std::placeholders::_1));
        FLIGHT_LOG_DEBUG(SENSOR, "扩展状态订阅器创建完成，QoS: best_effort");

        // 3. 订阅里程计信息
        FLIGHT_LOG_INFO(SENSOR, "创建里程计订阅器: odom");
        auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::QoS(100).best_effort().durability_volatile(),
            std::bind(&Test::odomCallback, &test, std::placeholders::_1));
        FLIGHT_LOG_DEBUG(SENSOR, "里程计订阅器创建完成，QoS: best_effort");

        //4. 订阅位置指令
        FLIGHT_LOG_INFO(MISSION, "创建位置指令订阅器: cmd");
        auto cmd_sub = node->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "cmd", rclcpp::QoS(5).reliable().durability_volatile(),
            std::bind(&Test::cmdCallback, &test, std::placeholders::_1));
        FLIGHT_LOG_DEBUG(MISSION, "位置指令订阅器创建完成，QoS: reliable");

        // 5. 订阅IMU数据
        FLIGHT_LOG_INFO(SENSOR, "创建IMU订阅器: /px4/imu (通过PX4 IMU Bridge转换)");
        auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
            "/px4/imu", rclcpp::QoS(100).best_effort().durability_volatile(),
            std::bind(&Test::imuCallback, &test, std::placeholders::_1));
        FLIGHT_LOG_DEBUG(SENSOR, "IMU订阅器创建完成，QoS: best_effort");

        // 6. 订阅遥控器数据（如果启用）
        rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr rc_sub;
        if (!param.takeoff_land.no_RC)
        {
            FLIGHT_LOG_INFO(SENSOR, "创建遥控器订阅器: /fmu/out/manual_control_setpoint");
            rc_sub = node->create_subscription<px4_msgs::msg::ManualControlSetpoint>(
                "/fmu/out/manual_control_setpoint", rclcpp::QoS(10).best_effort().durability_volatile(),
                std::bind(&Test::rcCallback, &test, std::placeholders::_1));
            FLIGHT_LOG_DEBUG(SENSOR, "遥控器订阅器创建完成，QoS: best_effort");
        } else {
            FLIGHT_LOG_WARN(SENSOR, "遥控器已禁用 (no_RC=true)");
        }

        // 7. 订阅电池状态（通过电池桥接器转换后的标准消息）
        FLIGHT_LOG_INFO(SENSOR, "创建电池状态订阅器: /px4/battery_state");
        auto bat_sub = node->create_subscription<sensor_msgs::msg::BatteryState>(
            "/px4/battery_state", rclcpp::QoS(10).best_effort().transient_local(),
            std::bind(&Test::batteryCallback, &test, std::placeholders::_1));
        FLIGHT_LOG_DEBUG(SENSOR, "电池状态订阅器创建完成，QoS: best_effort + transient_local");

        // 8. 订阅起飞降落命令
        FLIGHT_LOG_INFO(MISSION, "创建起飞降落订阅器: takeoff_land");
        auto takeoff_land_sub = node->create_subscription<quadrotor_msgs::msg::TakeoffLand>(
            "/takeoff_land", rclcpp::QoS(5).reliable().durability_volatile(),
            std::bind(&Test::takeoffLandCallback, &test, std::placeholders::_1));
        FLIGHT_LOG_DEBUG(MISSION, "起飞降落订阅器创建完成，QoS: reliable");
    #elif TEST_OPEN == 0
        FLIGHT_LOG_INFO(PX4CTRL, "创建正常模式订阅器...");
        
        // 1. 订阅飞控状态信息（仅接受状态）
        FLIGHT_LOG_INFO(SENSOR, "创建飞控状态订阅器: /fmu/out/vehicle_status_v1");
        auto state_sub = node->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", rclcpp::QoS(10).best_effort().durability_volatile(),
            std::bind(&State_Data_t::feed, &fsm.state_data, std::placeholders::_1));
        FLIGHT_LOG_DEBUG(SENSOR, "飞控状态订阅器创建完成，QoS: best_effort");

        // 2. 订阅扩展状态信息（仅接受状态）
        // 修改主题：/mavros/extended_state -> /fmu/out/vehicle_land_detected
        auto extended_state_sub = node->create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected", rclcpp::QoS(10).best_effort().durability_volatile(),
            std::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, std::placeholders::_1));

        // 3. 订阅里程计信息（接受里程计数据后提取并检查频率）
        auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::QoS(100).best_effort().durability_volatile(),
            std::bind(&Odom_Data_t::feed, &fsm.odom_data, std::placeholders::_1));

        //4. 订阅位置指令
        auto cmd_sub = node->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "cmd", rclcpp::QoS(5).reliable().durability_volatile(),
            std::bind(&Command_Data_t::feed, &fsm.cmd_data, std::placeholders::_1));

        // 5. 订阅IMU数据
        // 修改主题：/mavros/imu/data -> /px4/imu (通过PX4 IMU Bridge转换后的标准sensor_msgs::Imu消息)
        auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
            "/px4/imu", rclcpp::QoS(100).best_effort().durability_volatile(),
            std::bind(&Imu_Data_t::feed, &fsm.imu_data, std::placeholders::_1));

        // 6. 订阅遥控器数据（如果启用）
        rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr rc_sub;
        
        // 详细记录起飞降落参数配置
        FLIGHT_LOG_INFO(SENSOR, "📋 [参数检查] 起飞降落配置参数:");
        FLIGHT_LOG_INFO(SENSOR, "   - 自动起降启用: %s", param.takeoff_land.enable ? "是" : "否");
        FLIGHT_LOG_INFO(SENSOR, "   - 自动解锁启用: %s", param.takeoff_land.enable_auto_arm ? "是" : "否");
        FLIGHT_LOG_INFO(SENSOR, "   - 无遥控器模式: %s", param.takeoff_land.no_RC ? "是" : "否");
        FLIGHT_LOG_INFO(SENSOR, "   - 起飞高度: %.2fm", param.takeoff_land.height);
        FLIGHT_LOG_INFO(SENSOR, "   - 起降速度: %.2fm/s", param.takeoff_land.speed);
        
        FLIGHT_LOG_INFO(SENSOR, "🔧 [RC订阅] 开始检查遥控器数据订阅条件...");
        
        if (!param.takeoff_land.no_RC)
        {
            FLIGHT_LOG_INFO(SENSOR, "✅ [RC订阅] 遥控器数据订阅已启用 - 开始创建订阅器");
            FLIGHT_LOG_INFO(SENSOR, "📡 [RC订阅] 订阅话题: /fmu/out/manual_control_setpoint");
            FLIGHT_LOG_INFO(SENSOR, "⚙️ [RC订阅] QoS配置: best_effort, durability_volatile, 队列大小: 10");
            
            try {
                // 创建QoS配置 - 匹配PX4发布方的配置
                auto qos = rclcpp::QoS(10).best_effort().transient_local();
                FLIGHT_LOG_INFO(SENSOR, "🔧 [RC订阅] QoS配置详情:");
                FLIGHT_LOG_INFO(SENSOR, "   - 队列深度: 10");
                FLIGHT_LOG_INFO(SENSOR, "   - 可靠性: best_effort");
                FLIGHT_LOG_INFO(SENSOR, "   - 持久性: transient_local (匹配PX4发布方)");
                FLIGHT_LOG_INFO(SENSOR, "🎯 [RC订阅] QoS配置已修复，现在与PX4发布方匹配！");
                
                rc_sub = node->create_subscription<px4_msgs::msg::ManualControlSetpoint>(
                    "/fmu/out/manual_control_setpoint", qos,
                    std::bind(&RC_Data_t::feed, &fsm.rc_data, std::placeholders::_1));
                
                FLIGHT_LOG_INFO(SENSOR, "✅ [RC订阅] 遥控器订阅器创建成功");
                FLIGHT_LOG_INFO(SENSOR, "📊 [RC订阅] 订阅器状态: 已激活，等待RC数据...");
                FLIGHT_LOG_INFO(SENSOR, "⏱️ [RC订阅] RC数据超时阈值: %.1f秒", param.msg_timeout.rc);
                
                // 添加订阅器状态检查
                if (rc_sub) {
                    FLIGHT_LOG_INFO(SENSOR, "✅ [RC订阅] 订阅器指针有效");
                } else {
                    FLIGHT_LOG_ERROR(SENSOR, "❌ [RC订阅] 订阅器指针为空！");
                }
                
            } catch (const std::exception& e) {
                FLIGHT_LOG_ERROR(SENSOR, "❌ [RC订阅] 遥控器订阅器创建失败: %s", e.what());
                FLIGHT_LOG_ERROR(SENSOR, "🚨 [RC订阅] 这将导致RC数据超时，影响起飞功能！");
            }
        } else {
            FLIGHT_LOG_WARN(SENSOR, "⚠️ [RC订阅] 遥控器已禁用 (no_RC=true)");
            FLIGHT_LOG_WARN(SENSOR, "🔧 [RC订阅] 无遥控器模式配置 - 依赖自动解锁功能");
            FLIGHT_LOG_WARN(SENSOR, "📋 [RC订阅] 当前配置: 自动起降=%s, 自动解锁=%s", 
                           param.takeoff_land.enable ? "启用" : "禁用",
                           param.takeoff_land.enable_auto_arm ? "启用" : "禁用");
        }

        // 7. 订阅电池状态（通过电池桥接器转换后的标准消息）
        // 修改主题：/mavros/battery -> /px4/battery_state (通过电池桥接器转换)
        auto bat_sub = node->create_subscription<sensor_msgs::msg::BatteryState>(
            "/px4/battery_state", rclcpp::QoS(10).best_effort().transient_local(),
            std::bind(&Battery_Data_t::feed, &fsm.bat_data, std::placeholders::_1));

        // 8. 订阅起飞降落命令
        FLIGHT_LOG_INFO(PX4CTRL, "订阅起飞降落命令: /takeoff_land");
        auto takeoff_land_sub = node->create_subscription<quadrotor_msgs::msg::TakeoffLand>(
            "/takeoff_land", rclcpp::QoS(5).reliable().durability_volatile(),
            std::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, std::placeholders::_1));
        FLIGHT_LOG_DEBUG(PX4CTRL, "订阅起飞降落命令完成");
        
        // 9. 订阅电机输出数据
        FLIGHT_LOG_INFO(SENSOR, "创建电机输出订阅器: /fmu/out/actuator_outputs");
        fsm.actuator_outputs_sub = node->create_subscription<px4_msgs::msg::ActuatorOutputs>(
            "/fmu/out/actuator_outputs", rclcpp::QoS(10).best_effort().durability_volatile(),
            std::bind(&PX4CtrlFSM::actuator_outputs_callback, &fsm, std::placeholders::_1));
        FLIGHT_LOG_DEBUG(SENSOR, "电机输出订阅器创建完成，QoS: best_effort");
    #endif // TEST_OPEN

    // 创建 best effort QoS profile
    auto best_effort_qos = rclcpp::QoS(10).best_effort().durability_volatile();
    
    // 创建发布器
    FLIGHT_LOG_INFO(CONTROLLER, "创建控制输出发布器...");
    FLIGHT_LOG_INFO(CONTROLLER, "创建姿态设定点发布器: /fmu/in/vehicle_attitude_setpoint");
    fsm.ctrl_FCU_pub = node->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        "/fmu/in/vehicle_attitude_setpoint", 
        best_effort_qos);  // 使用 best effort QoS
    FLIGHT_LOG_DEBUG(CONTROLLER, "姿态设定点发布器创建完成，QoS: best_effort");
    
    FLIGHT_LOG_INFO(MISSION, "创建轨迹启动信号发布器: /traj_start_trigger");
    fsm.traj_start_trigger_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/traj_start_trigger", 10);//发布轨迹启动信号
    FLIGHT_LOG_DEBUG(MISSION, "轨迹启动信号发布器创建完成");
    
    FLIGHT_LOG_INFO(ROS2, "创建调试信息发布器: /debugPx4ctrl");
    fsm.debug_pub = node->create_publisher<quadrotor_msgs::msg::Px4ctrlDebug>(
        "/debugPx4ctrl", 10);//发布调试信息
    FLIGHT_LOG_DEBUG(ROS2, "调试信息发布器创建完成");

    // 创建服务客户端
    FLIGHT_LOG_INFO(PX4CTRL, "创建车辆命令发布器: /fmu/in/vehicle_command");
    fsm.vehicle_command_pub = node->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);
    FLIGHT_LOG_DEBUG(PX4CTRL, "车辆命令发布器创建完成");
    
    // 创建Offboard模式相关发布器
    FLIGHT_LOG_INFO(PX4CTRL, "创建Offboard控制模式发布器: /fmu/in/offboard_control_mode");
    fsm.offboard_control_mode_pub = node->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", best_effort_qos);
    FLIGHT_LOG_DEBUG(PX4CTRL, "Offboard控制模式发布器创建完成");
    
    FLIGHT_LOG_INFO(PX4CTRL, "创建轨迹设定点发布器: /fmu/in/trajectory_setpoint");
    fsm.trajectory_setpoint_pub = node->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", best_effort_qos);
    FLIGHT_LOG_DEBUG(PX4CTRL, "轨迹设定点发布器创建完成");

    // 检查遥控器状态
    if (param.takeoff_land.no_RC)
    {
        //遥控器已禁用，请注意安全！
        FLIGHT_LOG_WARN(PX4CTRL, "[PX4CTRL] Remote controller disabled, be careful!");
    }
    else
    {
        FLIGHT_LOG_INFO(PX4CTRL, "[PX4CTRL] Waiting for RC");//等待遥控器连接
        while (rclcpp::ok())
        {
            rclcpp::spin_some(node);
            rclcpp::Time current_time = node->now();
            if (fsm.rc_is_received(current_time))
            {
                FLIGHT_LOG_INFO(PX4CTRL, "[PX4CTRL] RC received.");//连接成功
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));//等待100ms
        }
    }

    // 等待连接到PX4飞控
    int trials = 0;
    while (rclcpp::ok() && !fsm.state_data.current_state.usb_connected)
    {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::seconds(1));
        if (trials++ > 5) // 等待时间不能超过5s
        {
            FLIGHT_LOG_WARN(PX4CTRL, "USB connection not detected, but continuing with PX4 communication...");
            break; // 添加break语句，避免无限循环
        }
    }


    // 主循环
    FLIGHT_LOG_INFO(PX4CTRL, "进入主循环，控制频率: %.0f Hz", param.ctrl_freq_max);
    FLIGHT_LOG_DEBUG(SYSTEM, "主循环启动，开始处理传感器数据和状态机");
    
    //**需要注意的是，如果循环中的处理时间超过了周期时间（1/频率）
    //**系统将无法达到设定的目标频率。因此在设置频率时要考虑系统的实际处理能力。
    rclcpp::Rate rate(param.ctrl_freq_max);  // 设置循环频率
    int loop_count = 0;
    
    while (rclcpp::ok())
    {
        auto loop_start = std::chrono::high_resolution_clock::now();
        
        rate.sleep();
        rclcpp::spin_some(node);  // 处理回调
        fsm.process();         // 运行状态机

        #if TEST_OPEN
            test.displayMessages();    // 调用显示函数
        #endif // TEST_OPEN
        
        // 每100次循环记录一次性能信息
        if (++loop_count % 100 == 0) {
            auto loop_end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start);
            FLIGHT_LOG_DEBUG(SYSTEM, "主循环性能: 第%d次循环，耗时: %ld μs", loop_count, duration.count());
        }
    }
    
    FLIGHT_LOG_INFO(PX4CTRL, "PX4Ctrl节点正常退出");

    return 0;
}
