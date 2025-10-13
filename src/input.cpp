#include "input.h"
#include "FlightLogger.h"

// RC_Data_t类用于处理遥控器输入数据
// 构造函数1-----------------------------------------------------------
RC_Data_t::RC_Data_t(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    // 初始化接收时间戳
    rcv_stamp = node_->now();

    // 初始化上一次的模式和档位值
    last_mode = -1.0;
    last_gear = -1.0;

    // 初始化飞行模式相关的标志位
    // 这些参数在无遥控器使用时非常重要!
    is_hover_mode = true;        // 是否处于悬停模式
    enter_hover_mode = false;    // 是否刚进入悬停模式
    is_command_mode = true;      // 是否处于命令模式
    enter_command_mode = false;  // 是否刚进入命令模式
    toggle_reboot = false;       // 重启开关
    
    // 初始化4个控制通道值为0
    for (int i = 0; i < 4; ++i)
    {
        ch[i] = 0.0;
    }
}

void RC_Data_t::feed(const px4_msgs::msg::ManualControlSetpoint::SharedPtr pMsg)
{
    // 记录RC数据接收时间
    rclcpp::Time now = node_->now();
    msg = *pMsg;
    rcv_stamp = now;

    // 使用ManualControlSetpoint的字段映射到RC通道
    // 通道1-4分别对应:横滚、俯仰、油门、偏航
    ch[0] = msg.roll;   // 横滚
    ch[1] = msg.pitch;  // 俯仰
    ch[2] = msg.throttle; // 油门
    ch[3] = msg.yaw;    // 偏航
    
    // 详细日志记录遥控器数据
    static int rc_log_counter = 0;
    static rclcpp::Time last_rc_time = now;
    static bool first_rc_received = false;
    static int total_rc_count = 0;
    
    total_rc_count++;
    
    // 记录第一次RC数据接收
    if (!first_rc_received) {
        FLIGHT_LOG_INFO(SENSOR, "🎉 [RC数据] 首次接收到遥控器数据！");
        FLIGHT_LOG_INFO(SENSOR, "📡 [RC数据] 数据源: %d, 时间戳: %.3f", 
                       msg.data_source, rcv_stamp.seconds());
        FLIGHT_LOG_INFO(SENSOR, "🔧 [RC数据] 回调函数正常工作，订阅器连接成功！");
        first_rc_received = true;
    }
    
    // 每100次接收记录一次统计信息
    if (total_rc_count % 100 == 0) {
        FLIGHT_LOG_INFO(SENSOR, "📊 [RC数据] 接收统计 - 总接收次数: %d", total_rc_count);
    }
    
    if (++rc_log_counter % 10 == 0) { // 每10次记录一次，进一步增加日志频率
        double time_since_last = (now - last_rc_time).seconds();
        FLIGHT_LOG_INFO(SENSOR, "📊 [RC数据] 控制通道 - Roll: %.3f, Pitch: %.3f, Throttle: %.3f, Yaw: %.3f", 
                     ch[0], ch[1], ch[2], ch[3]);
        FLIGHT_LOG_INFO(SENSOR, "🔧 [RC数据] 辅助通道 - Aux1: %.3f, Aux2: %.3f, Aux3: %.3f, Aux4: %.3f", 
                     msg.aux1, msg.aux2, msg.aux3, msg.aux4);
        FLIGHT_LOG_DEBUG(SENSOR, "⏱️ [RC数据] 时间信息 - 接收时间: %.3f, 距上次: %.3fs, 数据源: %d", 
                     rcv_stamp.seconds(), time_since_last, msg.data_source);
        last_rc_time = now;
    }
    
    // 设置死区,消除微小的摇杆抖动
    for (int i = 0; i < 4; i++)
    {
        if (ch[i] > DEAD_ZONE)
            ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (ch[i] < -DEAD_ZONE)
            ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            ch[i] = 0.0;
    }

    // 处理特殊通道 - 使用aux字段
    mode = msg.aux1;      // aux1:飞行模式选择
    gear = msg.aux2;       // aux2:档位切换
    reboot_cmd = msg.aux4; // aux4:重启命令

    check_validity();//检查数据有效性

    // 初始化上一次的模式、档位和重启命令值(第一次进入时)
    if (!have_init_last_mode)
    {
        have_init_last_mode = true;
        last_mode = mode;
    }
    if (!have_init_last_gear)
    {
        have_init_last_gear = true;
        last_gear = gear;
    }
    if (!have_init_last_reboot_cmd)
    {
        have_init_last_reboot_cmd = true;
        last_reboot_cmd = reboot_cmd;
    }

    //上次不是悬停模式，这次是悬停模式=进入悬停
    if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE)
        enter_hover_mode = true;
    // else
    //     enter_hover_mode = false;

    //模式为悬停模式，就是处于悬停模式
    if (mode > API_MODE_THRESHOLD_VALUE)
        is_hover_mode = true;
    else
        is_hover_mode = false;

    // 2 判断命令模式
    if (is_hover_mode)
    {
        if (last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE)
            enter_command_mode = true;
        else if (gear < GEAR_SHIFT_VALUE)
            enter_command_mode = false;

        if (gear > GEAR_SHIFT_VALUE)
            is_command_mode = true;
        else
            is_command_mode = false;
    }

    // 3 处理重启命令
    if (!is_hover_mode && !is_command_mode)
    {
                // 添加toggle_reboot的输出
        FLIGHT_LOG_INFO(SENSOR, "hhh");
        if (last_reboot_cmd < REBOOT_THRESHOLD_VALUE && reboot_cmd > REBOOT_THRESHOLD_VALUE)
            toggle_reboot = true;
        else
            toggle_reboot = false;
    }
    else
        toggle_reboot = false;

    // 更新上一次的模式、档位和重启命令值
    last_mode = mode;
    last_gear = gear;
    last_reboot_cmd = reboot_cmd;
}

void RC_Data_t::check_validity()
{
    if (mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1 && reboot_cmd >= -1.1 && reboot_cmd <= 1.1)
    {
        // 通过
    }
    else
    {
        FLIGHT_LOG_ERROR(SENSOR, "RC data validity check fail. mode=%f, gear=%f, reboot_cmd=%f", mode, gear, reboot_cmd);
    }
}

bool RC_Data_t::check_centered()
{
    bool centered = abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5;
    return centered;
}

// Odom_Data_t类用于处理里程计数据
// 构造函数2-----------------------------------------------------------
Odom_Data_t::Odom_Data_t(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    rcv_stamp = node_->now();;
    q.setIdentity();  // 初始化四元数为单位四元数
    recv_new_msg = false;
}

void Odom_Data_t::feed(const nav_msgs::msg::Odometry::SharedPtr pMsg)
{
    rclcpp::Time now = node_->now();

    msg = *pMsg;
    rcv_stamp = now;
    recv_new_msg = true;

    uav_utils::extract_odometry(pMsg, p, v, q, w);
    
    // 详细日志记录里程计数据
    static int odom_log_counter = 0;
    if (++odom_log_counter % 20 == 0) { // 每20次记录一次，进一步增加日志频率
        FLIGHT_LOG_INFO(SENSOR, "里程计数据 - 位置: [%.3f, %.3f, %.3f], 速度: [%.3f, %.3f, %.3f]", 
                     p(0), p(1), p(2), v(0), v(1), v(2));
        FLIGHT_LOG_INFO(SENSOR, "里程计数据 - 角速度: [%.3f, %.3f, %.3f]", 
                     w(0), w(1), w(2));
        FLIGHT_LOG_DEBUG(SENSOR, "里程计四元数: [%.3f, %.3f, %.3f, %.3f]", 
                     q.w(), q.x(), q.y(), q.z());
        FLIGHT_LOG_DEBUG(SENSOR, "里程计时间戳: %.3f, 接收时间: %.3f", 
                     msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, rcv_stamp.seconds());
    }

// #define VEL_IN_BODY
#ifdef VEL_IN_BODY /* Set to 1 if the velocity in odom topic is relative to current body frame, not to world frame.*/
    Eigen::Quaternion<double> wRb_q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Matrix3d wRb = wRb_q.matrix();
    v = wRb * v;

    static int count = 0;
    if (count++ % 500 == 0)
        ROS_WARN("VEL_IN_BODY!!!");
#endif

    // 检查频率
    // 修复频率检查逻辑：将初始化值从9999改为0，解决循环误报问题
    // 原因：原代码one_min_count=9999导致第一次检查时计数为10000，跳过告警检查
    // 但后续1秒内如果消息数量<100就会误报，造成持续告警
    // 修改后：正常统计1秒内消息数量，只有真正频率<100Hz时才告警
    static int one_min_count = 0;  // 修改：从9999改为0
    static rclcpp::Time last_clear_count_time = node_->now();
    if ( (now - last_clear_count_time).seconds() > 1.0 )
    {
        if ( one_min_count < 100 )
        {
            FLIGHT_LOG_WARN(SENSOR, "ODOM frequency seems lower than 100Hz, which is too low!");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count++;
}

//构造函数3-----------------------------------------------------------
Imu_Data_t::Imu_Data_t(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    rcv_stamp = node_->now();;
}

void Imu_Data_t::feed(const sensor_msgs::msg::Imu::SharedPtr pMsg)
{
    rclcpp::Time now = node_->now();

    msg = *pMsg;
    rcv_stamp = now;

    // 处理来自传感器的消息，提取角速度、线性加速度和方向数据
    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;
    
    // 详细日志记录IMU数据
    static int imu_log_counter = 0;
    if (++imu_log_counter % 30 == 0) { // 每30次记录一次
        FLIGHT_LOG_INFO(SENSOR, "IMU数据 - 角速度: [%.3f, %.3f, %.3f], 线性加速度: [%.3f, %.3f, %.3f]", 
                     w(0), w(1), w(2), a(0), a(1), a(2));
        FLIGHT_LOG_DEBUG(SENSOR, "IMU四元数: [%.3f, %.3f, %.3f, %.3f]", 
                     q.w(), q.x(), q.y(), q.z());
        FLIGHT_LOG_DEBUG(SENSOR, "IMU时间戳: %.3f, 接收时间: %.3f", 
                     msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, rcv_stamp.seconds());
    }

    // check the frequency
    // 修复频率检查逻辑：将初始化值从9999改为0，解决循环误报问题
    // 原因：原代码one_min_count=9999导致第一次检查时计数为10000，跳过告警检查
    // 但后续1秒内如果消息数量<100就会误报，造成持续告警
    // 修改后：正常统计1秒内消息数量，只有真正频率<100Hz时才告警
    static int one_min_count = 0;  // 修改：从9999改为0
    static rclcpp::Time last_clear_count_time = node_->now();;
    if ( (now - last_clear_count_time).seconds() > 1.0 )
    {
        if ( one_min_count < 100 )
        {
            //要求低于10ms才采样速度
            FLIGHT_LOG_WARN(SENSOR, "IMU frequency seems lower than 100Hz, which is too low!");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count++;
}

//构造函数4-----------------------------------------------------------
State_Data_t::State_Data_t(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
}

void State_Data_t::feed(const px4_msgs::msg::VehicleStatus::SharedPtr pMsg)
{
    current_state = *pMsg;
    
    // 使用静态计数器，避免频繁打印日志
    static int log_counter = 0;
    if (++log_counter % 50 == 0) {  // 每50次打印一次
        LISTENER_LOG_INFO("飞控状态更新 - 导航状态: %d, 解锁状态: %d", 
                         pMsg->nav_state, pMsg->arming_state);
    }
}

//构造函数5-----------------------------------------------------------
ExtendedState_Data_t::ExtendedState_Data_t(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
}

void ExtendedState_Data_t::feed(const px4_msgs::msg::VehicleLandDetected::SharedPtr pMsg)
{
    current_extended_state = *pMsg;
    
    // 使用静态计数器，避免频繁打印日志
    // 扩展状态信息频率较低，每20次记录一次
    static int log_counter = 0;
    if (++log_counter % 20 == 0) {
        LISTENER_LOG_INFO("扩展状态更新 - 着陆检测: %s, 着陆状态: %s", 
                         pMsg->landed ? "已着陆" : "未着陆",
                         pMsg->in_ground_effect ? "地面效应中" : "正常飞行");
    }
}

//构造函数6-----------------------------------------------------------
Command_Data_t::Command_Data_t(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    rcv_stamp = node_->now();;
}

void Command_Data_t::feed(const quadrotor_msgs::msg::PositionCommand::SharedPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = node_->now();

    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;

    // PositionCommand消息中没有jerk字段，设置为0
    j(0) = 0.0;
    j(1) = 0.0;
    j(2) = 0.0;

    // std::cout << "j1=" << j.transpose() << std::endl;

    yaw = uav_utils::normalize_angle(msg.yaw);
    yaw_rate = msg.yaw_dot;
    
    // 使用静态计数器，避免频繁打印日志
    // cmd主题频率较高，每100次记录一次
    static int log_counter = 0;
    if (++log_counter % 100 == 0) {
        LISTENER_LOG_INFO("位置指令更新 - 位置: [%.3f, %.3f, %.3f], 速度: [%.3f, %.3f, %.3f], 偏航: %.3f", 
                         p(0), p(1), p(2), v(0), v(1), v(2), yaw);
    }
}

//构造函数7-----------------------------------------------------------
Battery_Data_t::Battery_Data_t(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    rcv_stamp = node_->now();;
}

void Battery_Data_t::feed(const sensor_msgs::msg::BatteryState::SharedPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = node_->now();

    double voltage = 0;
    for (size_t i = 0; i < pMsg->cell_voltage.size(); ++i)
    {
        voltage += pMsg->cell_voltage[i];
    }
    volt = 0.8 * volt + 0.2 * voltage; // Naive LPF, cell_voltage has a higher frequency

    // volt = 0.8 * volt + 0.2 * pMsg->voltage; // Naive LPF
    percentage = pMsg->percentage;

    static rclcpp::Time last_print_t = node_->now();
    if (percentage > 0.05)
    {
        if ((rcv_stamp - last_print_t).seconds() > 10)
        {
            FLIGHT_LOG_INFO(SENSOR, "Voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
    else
    {
        if ((rcv_stamp - last_print_t).seconds() > 1)
        {
            // ROS_ERROR("[px4ctrl] Dangerous! voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
}

//构造函数8-----------------------------------------------------------
Takeoff_Land_Data_t::Takeoff_Land_Data_t(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    rcv_stamp = node_->now();
}

void Takeoff_Land_Data_t::feed(const quadrotor_msgs::msg::TakeoffLand::SharedPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = node_->now();

    triggered = true;
    takeoff_land_cmd = pMsg->takeoff_land_cmd;
    
    // 详细记录起飞降落命令接收情况
    FLIGHT_LOG_INFO(MISSION, "🎯 [正常模式] 起飞降落命令接收 - 命令类型: %d", pMsg->takeoff_land_cmd);
    
    // 根据命令类型输出详细信息
    switch(pMsg->takeoff_land_cmd) {
        case 1:
            FLIGHT_LOG_INFO(MISSION, "🚀 [正常模式] 起飞命令已接收 - 触发状态机处理");
            FLIGHT_LOG_INFO(MISSION, "📋 [正常模式] 起飞命令详情 - 命令类型: %d", 
                       pMsg->takeoff_land_cmd);
            FLIGHT_LOG_INFO(MISSION, "🔄 [正常模式] 状态机将检查起飞条件并执行起飞序列");
            break;
        case 2:
            FLIGHT_LOG_INFO(MISSION, "🛬 [正常模式] 降落命令已接收 - 触发状态机处理");
            FLIGHT_LOG_INFO(MISSION, "📋 [正常模式] 降落命令详情 - 命令类型: %d", 
                       pMsg->takeoff_land_cmd);
            FLIGHT_LOG_INFO(MISSION, "🔄 [正常模式] 状态机将执行安全降落序列");
            break;
        default:
            FLIGHT_LOG_WARN(MISSION, "⚠️ [正常模式] 未知的起飞降落命令: %d", pMsg->takeoff_land_cmd);
            break;
    }
    
    // 记录触发状态
    FLIGHT_LOG_INFO(MISSION, "✅ [正常模式] 起飞降落数据触发标志已设置: triggered=%s", triggered ? "true" : "false");
}
