#include "controller.h"
#include "FlightLogger.h"

using namespace std;

// 从四元数计算偏航角
double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
    // 使用arctan2计算偏航角，范围[-pi,pi]
    double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
    
    // 详细日志记录偏航角计算
    static int yaw_log_counter = 0;
    if (++yaw_log_counter % 50 == 0) { // 每50次记录一次
        FLIGHT_LOG_DEBUG(CONTROLLER, "偏航角计算 - 四元数: [%.3f, %.3f, %.3f, %.3f], 偏航角: %.3f", 
                     q.w(), q.x(), q.y(), q.z(), yaw);
    }
    
    return yaw;
}

// 构造函数
LinearControl::LinearControl(Parameter_t &param, const std::shared_ptr<rclcpp::Node>& node) 
    : param_(param), node_(node)
{
    resetThrustMapping();  // 初始化推力映射参数
    
    // 详细日志记录控制器初始化
    FLIGHT_LOG_INFO(CONTROLLER, "LinearControl控制器初始化完成");
    FLIGHT_LOG_INFO(CONTROLLER, "PID参数 - Kp: [%.3f, %.3f, %.3f], Kv: [%.3f, %.3f, %.3f]", 
                param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2,
                param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2);
    FLIGHT_LOG_INFO(CONTROLLER, "推力映射参数 - 悬停百分比: %.3f, 重力: %.3f", 
                param_.thr_map.hover_percentage, param_.gra);
}

/* 
计算控制输出：推力和姿态四元数
输入：
- des: 期望状态（绝对期望）
- odom: 里程计数据
- imu: IMU数据
- u: 控制输出（相对期望）
*/
quadrotor_msgs::msg::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
    // 详细日志记录控制器输入
    static int control_log_counter = 0;
    if (++control_log_counter % 20 == 0) { // 每20次记录一次
        FLIGHT_LOG_INFO(CONTROLLER, "控制器输入 - 期望位置: [%.3f, %.3f, %.3f], 期望速度: [%.3f, %.3f, %.3f]", 
                     des.p(0), des.p(1), des.p(2), des.v(0), des.v(1), des.v(2));
        FLIGHT_LOG_INFO(CONTROLLER, "控制器输入 - 当前位置: [%.3f, %.3f, %.3f], 当前速度: [%.3f, %.3f, %.3f]", 
                     odom.p(0), odom.p(1), odom.p(2), odom.v(0), odom.v(1), odom.v(2));
    }
    
    /* 计算期望加速度 */
    Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
    Eigen::Vector3d Kp,Kv;
    // 获取PID参数
    Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
    Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
    
    // 详细日志记录PID参数
    if (control_log_counter % 20 == 0) {
        FLIGHT_LOG_DEBUG(CONTROLLER, "PID参数 - Kp: [%.3f, %.3f, %.3f], Kv: [%.3f, %.3f, %.3f]", 
                     Kp(0), Kp(1), Kp(2), Kv(0), Kv(1), Kv(2));
    }
    
    // 计算期望加速度：前馈项 + 速度反馈 + 位置反馈
    des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);
    des_acc += Eigen::Vector3d(0,0,param_.gra);  // 补偿重力
    
    // 详细日志记录加速度计算
    if (control_log_counter % 20 == 0) {
        FLIGHT_LOG_DEBUG(CONTROLLER, "加速度计算 - 位置误差: [%.3f, %.3f, %.3f], 速度误差: [%.3f, %.3f, %.3f]", 
                     (des.p - odom.p)(0), (des.p - odom.p)(1), (des.p - odom.p)(2),
                     (des.v - odom.v)(0), (des.v - odom.v)(1), (des.v - odom.v)(2));
        FLIGHT_LOG_DEBUG(CONTROLLER, "期望加速度: [%.3f, %.3f, %.3f]", des_acc(0), des_acc(1), des_acc(2));
    }

    // 计算期望推力
    u.thrust = computeDesiredCollectiveThrustSignal(des_acc);//用到z加速度
    
    // 详细日志记录推力计算
    if (control_log_counter % 20 == 0) {
        FLIGHT_LOG_DEBUG(CONTROLLER, "推力计算 - 期望推力: %.3f, 推力映射系数: %.3f", 
                     u.thrust, thr2acc_);
    }

    // 计算期望姿态
    double roll,pitch,/*yaw,*/yaw_imu;
    double yaw_odom = fromQuaternion2yaw(odom.q);
    double sin = std::sin(yaw_odom);
    double cos = std::cos(yaw_odom);
    // 根据期望加速度计算期望姿态角
    roll = (des_acc(0) * sin - des_acc(1) * cos )/ param_.gra;
    pitch = (des_acc(0) * cos + des_acc(1) * sin )/ param_.gra;
    
    yaw_imu = fromQuaternion2yaw(imu.q);
    
    // 详细日志记录姿态角计算
    if (control_log_counter % 20 == 0) {
        FLIGHT_LOG_DEBUG(CONTROLLER, "姿态角计算 - 横滚: %.3f, 俯仰: %.3f, 偏航: %.3f", 
                     roll, pitch, des.yaw);
        FLIGHT_LOG_DEBUG(CONTROLLER, "当前偏航角 - 里程计: %.3f, IMU: %.3f", yaw_odom, yaw_imu);
    }

    // 构建期望姿态四元数
    Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
    // 计算期望姿态四元数(imu当前姿态*(里程计当前姿态的逆*期望里程计姿态))，绝对姿态掺入相对姿态
    u.q = imu.q * odom.q.inverse() * q;
    
    // 详细日志记录四元数计算
    if (control_log_counter % 20 == 0) {
        FLIGHT_LOG_DEBUG(CONTROLLER, "四元数计算 - 期望四元数: [%.3f, %.3f, %.3f, %.3f]", 
                     u.q.w(), u.q.x(), u.q.y(), u.q.z());
    }

    // 更新调试信息
    debug_msg_.des_v_x = des.v(0);
    debug_msg_.des_v_y = des.v(1);
    debug_msg_.des_v_z = des.v(2);
    
    debug_msg_.des_a_x = des_acc(0);
    debug_msg_.des_a_y = des_acc(1);
    debug_msg_.des_a_z = des_acc(2);
    
    debug_msg_.des_q_x = u.q.x();
    debug_msg_.des_q_y = u.q.y();
    debug_msg_.des_q_z = u.q.z();
    debug_msg_.des_q_w = u.q.w();
    
    debug_msg_.des_thr = u.thrust;

    // 存储推力数据用于推力-加速度映射估计
    timed_thrust_.push(std::pair<rclcpp::Time, double>(node_->now(), u.thrust));
    while (timed_thrust_.size() > 100)
    {
        timed_thrust_.pop();
    }
    
    // 详细日志记录控制器输出
    if (control_log_counter % 20 == 0) {
        FLIGHT_LOG_INFO(CONTROLLER, "控制器输出 - 推力: %.3f, 四元数: [%.3f, %.3f, %.3f, %.3f]", 
                     u.thrust, u.q.w(), u.q.x(), u.q.y(), u.q.z());
        FLIGHT_LOG_DEBUG(CONTROLLER, "推力数据队列大小: %zu", timed_thrust_.size());
    }
    
    return debug_msg_;
}

/*
计算期望的归一化总推力信号
输入：期望加速度
*/
double 
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
    double throttle_percentage(0.0);
    
    // 使用推力到加速度的映射系数计算油门百分比
    throttle_percentage = des_acc(2) / thr2acc_;
    
    // 详细日志记录推力信号计算
    static int thrust_log_counter = 0;
    if (++thrust_log_counter % 50 == 0) { // 每50次记录一次
        FLIGHT_LOG_DEBUG(CONTROLLER, "推力信号计算 - Z轴加速度: %.3f, 推力映射系数: %.3f, 油门百分比: %.3f", 
                     des_acc(2), thr2acc_, throttle_percentage);
    }

    return throttle_percentage;
}

/*
估计推力模型参数
使用递归最小二乘法(RLS)估计推力到加速度的映射关系
*/
bool 
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t & /* param */)
{
    rclcpp::Time t_now = node_->now();
    
    // 详细日志记录推力模型估计
    static int thrust_model_log_counter = 0;
    if (++thrust_model_log_counter % 100 == 0) { // 每100次记录一次
        FLIGHT_LOG_DEBUG(CONTROLLER, "推力模型估计 - 估计加速度: [%.3f, %.3f, %.3f], 队列大小: %zu", 
                     est_a(0), est_a(1), est_a(2), timed_thrust_.size());
    }
    
    while (timed_thrust_.size() >= 1)
    {
        // 选择35~45ms前的数据
        std::pair<rclcpp::Time, double> t_t = timed_thrust_.front();
        double time_passed = (t_now - t_t.first).seconds();
        if (time_passed > 0.045) // 45ms
        {
            timed_thrust_.pop();
            continue;
        }
        if (time_passed < 0.035) // 35ms
        {
            return false;
        }

        // 递归最小二乘法(RLS)算法
        double thr = t_t.second;
        timed_thrust_.pop();
        
        // 模型：est_a(2) = thr2acc_ * thr
        double gamma = 1 / (rho2_ + thr * P_ * thr);
        double K = gamma * P_ * thr;
        double old_thr2acc = thr2acc_;
        thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
        P_ = (1 - K * thr) * P_ / rho2_;
        
        // 详细日志记录RLS算法更新
        if (thrust_model_log_counter % 100 == 0) {
            FLIGHT_LOG_DEBUG(CONTROLLER, "RLS算法更新 - 时间差: %.3f, 推力: %.3f, 旧映射系数: %.3f, 新映射系数: %.3f", 
                         time_passed, thr, old_thr2acc, thr2acc_);
        }

        return true;
    }
    return false;
}

// 重置推力映射参数
void 
LinearControl::resetThrustMapping(void)
{
    // 使用悬停推力百分比初始化推力到加速度的映射
    double old_thr2acc = thr2acc_;
    thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
    P_ = 1e6;  // 初始化协方差矩阵
    
    // 详细日志记录推力映射重置
    FLIGHT_LOG_INFO(CONTROLLER, "推力映射重置 - 旧映射系数: %.3f, 新映射系数: %.3f, 协方差矩阵: %.3f", 
                 old_thr2acc, thr2acc_, P_);
    FLIGHT_LOG_INFO(CONTROLLER, "推力映射参数 - 重力: %.3f, 悬停百分比: %.3f", 
                 param_.gra, param_.thr_map.hover_percentage);
}
