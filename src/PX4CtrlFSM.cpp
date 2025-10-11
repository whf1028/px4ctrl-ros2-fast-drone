#include "PX4CtrlFSM.h"
#include "FlightLogger.h"

using namespace std;
using namespace uav_utils;

// 构造函数,初始化各个组件
PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, LinearControl &controller_, rclcpp::Node::SharedPtr node) 
    : param(param_),           // 参数配置
      controller(controller_), // 控制器
      node_(node),            // ROS2节点
      rc_data(node),          // 遥控器数据
      state_data(node),       // 状态数据
      extended_state_data(node), // 扩展状态数据
      odom_data(node),        // 里程计数据
      imu_data(node),         // IMU数据
      cmd_data(node),         // 指令数据
      bat_data(node),         // 电池数据
      takeoff_land_data(node) // 起飞降落数据
{
    state = MANUAL_CTRL;      // 初始状态为手动控制
	set_offboard_flag = false;  // 初始化起飞降落标志
	takeoff_land_data.triggered = false;
	RCLCPP_INFO(node_->get_logger(), "\033[32m[px4ctrl] NONE --> MANUAL_CTRL(L1)\033[0m");
    hover_pose.setZero();     // 悬停位置初始化为零点
    
    FLIGHT_LOG_INFO(FLIGHT_PHASE, "PX4CtrlFSM初始化完成，初始状态: MANUAL_CTRL");
    FLIGHT_LOG_DEBUG(SYSTEM, "状态机组件初始化: 遥控器、状态数据、里程计、IMU、指令、电池、起飞降落");
}

/* 
        有限状态机(FSM)的状态转换图:

	      系统启动
	            |
	            |
	            v
	----- > 手动控制 <-----------------
	|         ^   |    \                 |
	|         |   |     \                |
	|         |   |      > 自动起飞      |
	|         |   |        /             |
	|         |   |       /              |
	|         |   |      /               |
	|         |   v     /                |
	|       自动悬停 <                   |
	|         ^   |  \  \                |
	|         |   |   \  \               |
	|         |	  |    > 自动降落 -------
	|         |   |
	|         |   v
	-------- 指令控制

*/

void PX4CtrlFSM::process()
{
    // 获取当前时间
    rclcpp::Time now_time = node_->now();
    Controller_Output_t u;                // 控制输出
    Desired_State_t des(odom_data);      // 期望状态
    bool rotor_low_speed_during_land = false; // 降落时电机是否低速运行标志
    rclcpp::Rate rate4takeoff(param.ctrl_freq_max);  // 设置循环频率

	// 每隔1秒输出一次
	static rclcpp::Time last_print_time = node_->now();
	if ((now_time - last_print_time).seconds() > 2.0)
	{
		std::string state_str;
		switch(state)
		{
			case MANUAL_CTRL:
				state_str = "MANUAL_CTRL";
				break;
			case AUTO_HOVER:
				state_str = "AUTO_HOVER"; 
				break;
			case CMD_CTRL:
				state_str = "CMD_CTRL";
				break;
			case AUTO_TAKEOFF:
				state_str = "AUTO_TAKEOFF";
				break;
			case AUTO_LAND:
				state_str = "AUTO_LAND";
				break;
			default:
				state_str = "UNKNOWN";
		}
        std::string system_status_str;
        switch(state_data.current_state.system_type) {
            case 0:
                system_status_str = "未初始化";
                break;
            case 1:
                system_status_str = "系统启动中";
                break;
            case 2:
                system_status_str = "校准中";
                break;
            case 3:
                system_status_str = "待命状态";
                break;
            case 4:
                system_status_str = "正常运行";
                break;
            case 5:
                system_status_str = "危险状态";
                break;
            case 6:
                system_status_str = "紧急状态";
                break;
            case 7:
                system_status_str = "关机状态";
                break;
            default:
                system_status_str = "未知状态";
        }

        RCLCPP_INFO(node_->get_logger(), 
            "\n=================== PX4 状态信息 ===================\n"
            "USB连接: %s\n"
            "解锁状态: %d\n"
            "导航状态: %d\n"
            "系统状态: %s\n"
            "当前模式: %s\n"
            "================================================",
            state_data.current_state.usb_connected ? "已连接" : "未连接",
            state_data.current_state.arming_state, 
            state_data.current_state.nav_state,
            system_status_str.c_str(),  // 使用转换后的中文状态描述
            state_str.c_str()
        );

		last_print_time = now_time;
	}

	// STEP1: 状态机运行
	// 状态机包含以下状态:
	// MANUAL_CTRL: 手动控制模式,完全由遥控器控制
	// AUTO_HOVER: 自动悬停模式,保持当前位置和高度
	// CMD_CTRL: 指令控制模式,执行位置/速度等指令
	// AUTO_TAKEOFF: 自动起飞模式
	// AUTO_LAND: 自动降落模式
	switch (state)
	{
	case MANUAL_CTRL:
	{
		if (rc_data.enter_hover_mode) // 尝试跳转到AUTO_HOVER
		{
			RCLCPP_INFO(node_->get_logger(), "\033[32m333\033[0m");
			if(!odom_is_received(now_time))
			{
				// 拒绝AUTO_HOVER(L2),因为没有里程计数据!
				RCLCPP_ERROR(node_->get_logger(), "Reject AUTO_HOVER(L2). No odom!");
				break;
			}
			if(cmd_is_received(now_time))
			{
				// 拒绝AUTO_HOVER(L2),因为你在进入AUTO_HOVER之前发送了指令,这是不允许的,请立即停止发送指令!
				RCLCPP_ERROR(node_->get_logger(), "Reject AUTO_HOVER(L2). You are sending commands before toggling into AUTO_HOVER, which is not allowed. Stop sending commands now!");
				break;
			}
			if(odom_data.v.norm() > 3.0)
			{
				//OK
				// 拒绝AUTO_HOVER(L2),因为里程计数据表明无人机速度大于3m/s,这可能表明定位模块出现了问题!
				RCLCPP_ERROR(node_->get_logger(), "Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
				break;
			}
            if(set_offboard_flag || toggle_offboard_mode(true))// 切换到offboard模式
			{
				RCLCPP_INFO(node_->get_logger(), "\033[32m111\033[0m");
				
				if (state_data.current_state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
				{
					RCLCPP_INFO(node_->get_logger(), "\033[32m222\033[0m");

					// 切换到自动悬停模式
					state = AUTO_HOVER;// 切换到自动悬停模式
					controller.resetThrustMapping();// 重置推力映射参数
					set_hov_with_odom();// 设置悬停位置
					set_offboard_flag = false;
					rc_data.enter_hover_mode = false;
					RCLCPP_INFO(node_->get_logger(), "\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[0m");
					
					FLIGHT_LOG_INFO(FLIGHT_PHASE, "状态转换: MANUAL_CTRL -> AUTO_HOVER");
					FLIGHT_LOG_DEBUG(CONTROLLER, "控制器推力映射重置完成");
					FLIGHT_LOG_DEBUG(FLIGHT_PHASE, "悬停位置设置完成，基于当前里程计数据");
				}
				else
				{
					RCLCPP_INFO(node_->get_logger(), "\033[32mIt's OK!Wait for offboard mode...\033[0m");
					break;
				}
			}
			else
			{
				RCLCPP_INFO(node_->get_logger(), "\033[31mReject AUTO_HOVER. Failed to toggle offboard mode!\033[0m");
				break;
			}
		}

		else if (param.takeoff_land.enable && takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::TAKEOFF) // Try to jump to AUTO_TAKEOFF
		{

			if (!odom_is_received(now_time))// 拒绝AUTO_TAKEOFF,因为没有里程计数据!
			{
				takeoff_land_data.triggered = false;
				RCLCPP_ERROR(node_->get_logger(), "Reject AUTO_TAKEOFF. No odom!");
				break;
			}
			if (cmd_is_received(now_time))  // 拒绝AUTO_TAKEOFF,因为你正在进入AUTO_TAKEOFF之前发送指令,这是不允许的,请立即停止发送指令!
			{
				takeoff_land_data.triggered = false;
				RCLCPP_ERROR(node_->get_logger(), "Reject AUTO_TAKEOFF. You are sending commands before toggling into AUTO_TAKEOFF, which is not allowed. Stop sending commands now!");
				break;
			}
			if (odom_data.v.norm() > 0.1)   // 拒绝AUTO_TAKEOFF,因为里程计数据表明无人机速度大于0.1m/s,这是不允许的!
			{
				takeoff_land_data.triggered = false;
				RCLCPP_ERROR(node_->get_logger(), "Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff is not allowed!", odom_data.v.norm());
				break;
			}
			if (!get_landed())				// 拒绝AUTO_TAKEOFF,因为起飞降落检测器说无人机现在没有降落!
			{
				takeoff_land_data.triggered = false;
				RCLCPP_ERROR(node_->get_logger(), "Reject AUTO_TAKEOFF. land detector says that the drone is not landed now!");
				break;
			}
			if (rc_is_received(now_time))   // 检查遥控器是否连接
			{
				if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered())
				{
					// 拒绝AUTO_TAKEOFF,如果遥控器没有连接或者没有处于自动悬停模式或指令控制模式,或者摇杆没有居中,请重新起飞!
					RCLCPP_ERROR(node_->get_logger(), "Reject AUTO_TAKEOFF. If you have your RC connected, keep its switches at \"auto hover\" and \"command control\" states, and all sticks at the center, then takeoff again.");
					while (rclcpp::ok())
					{
						rate4takeoff.sleep();
						// rclcpp::sleep_for(std::chrono::milliseconds(10));
						rclcpp::spin_some(node_);
						if (rc_data.is_hover_mode && rc_data.is_command_mode && rc_data.check_centered())
						{
							// 可以再次起飞
							takeoff_land_data.triggered = false;
							RCLCPP_INFO(node_->get_logger(), "\033[32m[px4ctrl] OK, you can takeoff again.\033[0m");
							break;
						}
					}
					break;
				}
			}

			// 切换到offboard模式
			if (set_offboard_flag || toggle_offboard_mode(true))
			{
				if(state_data.current_state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
				{
					state = AUTO_TAKEOFF;
					controller.resetThrustMapping();
					set_start_pose_for_takeoff_land(odom_data);	
					// 如果启用了自动解锁功能
					if (param.takeoff_land.enable_auto_arm)
					{
						toggle_arm_disarm(true);
						FLIGHT_LOG_INFO(FLIGHT_PHASE, "自动解锁功能已启用，发送解锁命令");
					}
					takeoff_land.toggle_takeoff_land_time = now_time;
					set_offboard_flag = false;
					takeoff_land_data.triggered = false;
					
					FLIGHT_LOG_INFO(FLIGHT_PHASE, "状态转换: MANUAL_CTRL -> AUTO_TAKEOFF");
					FLIGHT_LOG_DEBUG(CONTROLLER, "控制器推力映射重置完成");
					FLIGHT_LOG_DEBUG(FLIGHT_PHASE, "起飞起始位置设置完成");

					RCLCPP_INFO(node_->get_logger(), "\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF\033[0m");
				}
				else
				{
					RCLCPP_INFO(node_->get_logger(), "\033[32mIt's OK!Wait for offboard mode...\033[0m");
					break;
				}

			}
			else
			{
				RCLCPP_INFO(node_->get_logger(), "\033[31mReject AUTO_TAKEOFF. Failed to toggle offboard mode!\033[0m");
				break;
			}
		}

		if (rc_data.toggle_reboot) // Try to reboot. EKF2 based PX4 FCU requires reboot when its state estimator goes wrong.
		{
			if (state_data.current_state.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
			{
				RCLCPP_ERROR(node_->get_logger(), "Reject reboot! Disarm the drone first!");// 拒绝重启! 先让无人机解锁!
				break;
			}
			rc_data.toggle_reboot = false;
			reboot_FCU();
		}

		break;
	}

	case AUTO_HOVER:
	{
		// 如果遥控器没有进入悬停模式或者没有收到里程计数据
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;// 切换到手动控制模式
			toggle_offboard_mode(false);// 关闭offboard模式

			RCLCPP_WARN(node_->get_logger(), "AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
			FLIGHT_LOG_WARN(FLIGHT_PHASE, "状态转换: AUTO_HOVER -> MANUAL_CTRL (遥控器未进入悬停模式或里程计数据丢失)");
			if (!rc_data.is_hover_mode) {
				FLIGHT_LOG_DEBUG(SENSOR, "遥控器未进入悬停模式");
			}
			if (!odom_is_received(now_time)) {
				FLIGHT_LOG_WARN(SENSOR, "里程计数据超时，时间: %.3f秒", (now_time - odom_data.rcv_stamp).seconds());
			}
		}
		// 如果遥控器进入指令模式并且收到指令数据
		else if (rc_data.is_command_mode && cmd_is_received(now_time))
		{
			if (state_data.current_state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
			{
				state = CMD_CTRL;// 切换到指令控制模式
				des = get_cmd_des();// 获取指令期望状态
				RCLCPP_INFO(node_->get_logger(), "\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[0m");
			}
		}
		// 如果起飞降落数据触发并且起飞降落指令为降落
		else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::LAND)
		{
			// 如果起飞降落数据触发并且起飞降落指令为降落
			state = AUTO_LAND;// 切换到自动降落模式
			set_start_pose_for_takeoff_land(odom_data);// 设置起飞降落起始位置

			RCLCPP_INFO(node_->get_logger(), "\033[32m[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[0m");
		}
		else
		{
			set_hov_with_rc();// 设置悬停位置
			des = get_hover_des();// 获取悬停期望状态
			if ((rc_data.enter_command_mode) ||
				(takeoff_land.delay_trigger.first && now_time > takeoff_land.delay_trigger.second))
			{
				takeoff_land.delay_trigger.first = false;
				publish_trigger(odom_data.msg);
				RCLCPP_INFO(node_->get_logger(), "\033[32m[px4ctrl] TRIGGER sent, allow user command.\033[0m");
			}

			// cout << "des.p=" << des.p.transpose() << endl;
		}

		break;
	}
	case CMD_CTRL:
	{
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			RCLCPP_WARN(node_->get_logger(), "From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
		}
		else if (!rc_data.is_command_mode || !cmd_is_received(now_time))
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			RCLCPP_INFO(node_->get_logger(), "From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
		}
		else
		{
			des = get_cmd_des();
		}

		if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::LAND)
		{
			RCLCPP_ERROR(node_->get_logger(), "Reject AUTO_LAND, which must be triggered in AUTO_HOVER. \
					Stop sending control commands for longer than %fs to let px4ctrl return to AUTO_HOVER first.",
					  param.msg_timeout.cmd);
		}

		break;
	}

	case AUTO_TAKEOFF:
	{
		//等待起飞
		if ((now_time - takeoff_land.toggle_takeoff_land_time).seconds() < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) // Wait for several seconds to warn prople.
		{ 
			des = get_rotor_speed_up_des(now_time);
		}
		else if (odom_data.p(2) >= (takeoff_land.start_pose(2) + param.takeoff_land.height)) // reach the desired height（起飞高度）
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			RCLCPP_INFO(node_->get_logger(), "\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[0m");

			takeoff_land.delay_trigger.first = true;
			takeoff_land.delay_trigger.second = now_time + rclcpp::Duration::from_seconds(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
		}
		else
		{
			des = get_takeoff_land_des(param.takeoff_land.speed);
		}

		break;
	}

	case AUTO_LAND:
	{
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			RCLCPP_WARN(node_->get_logger(), "From AUTO_LAND to MANUAL_CTRL(L1)!");
		}
		else if (!rc_data.is_command_mode)
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			RCLCPP_INFO(node_->get_logger(), "From AUTO_LAND to AUTO_HOVER(L2)!");
		}
		else if (!get_landed())
		{
			des = get_takeoff_land_des(-param.takeoff_land.speed);
		}
		else
		{
			rotor_low_speed_during_land = true;

			static bool print_once_flag = true;
			if (print_once_flag)
			{
				RCLCPP_INFO(node_->get_logger(), "\033[32m[px4ctrl] Wait for abount 10s to let the drone arm.\033[0m");
				print_once_flag = false;
			}

                        if (extended_state_data.current_extended_state.landed) // PX4 allows disarm after this
			{
				static double last_trial_time = 0; // Avoid too frequent calls
				if (now_time.seconds() - last_trial_time > 1.0)
				{
					if (toggle_arm_disarm(false)) // disarm
					{
						print_once_flag = true;
						state = MANUAL_CTRL;
						toggle_offboard_mode(false); // toggle off offboard after disarm
						RCLCPP_INFO(node_->get_logger(), "\033[32m[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)\033[0m");
					}

					last_trial_time = now_time.seconds();
				}
			}
		}

		break;
	}

	default:
		break;
	}

	// STEP2: 估计推力模型
	if (state == AUTO_HOVER || state == CMD_CTRL)
	{
		controller.estimateThrustModel(imu_data.a,param);
	}

	// STEP3: 解决并更新新的控制命令
	if (rotor_low_speed_during_land) // 在自动起飞开始时使用
	{
		motors_idling(imu_data, u);
	}
	else
	{
		debug_msg = controller.calculateControl(des, odom_data, imu_data, u);
		debug_msg.header.stamp = now_time;
		debug_pub->publish(debug_msg);
	}

	// STEP4: 发布控制命令到mavros
	if (param.use_bodyrate_ctrl)
	{
		publish_bodyrate_ctrl(u, now_time);
	}
	else
	{
		publish_attitude_ctrl(u, now_time);
	}

	// STEP5: 检测无人机是否降落
	land_detector(state, des, odom_data);
	// cout << takeoff_land.landed << " ";
	// fflush(stdout);

	// STEP6: 清除超出生命周期的标志
	// rc_data.enter_hover_mode = false;
	rc_data.enter_command_mode = false;
	// rc_data.toggle_reboot = false;
	// takeoff_land_data.triggered = false;
}

void PX4CtrlFSM::motors_idling(const Imu_Data_t &imu, Controller_Output_t &u)
{
	u.q = imu.q;
	u.bodyrates = Eigen::Vector3d::Zero();
	u.thrust = 0.04;
}

// 检测无人机是否降落,如果降落则设置takeoff_land.landed为true
void PX4CtrlFSM::land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom)
{
	static State_t last_state = State_t::MANUAL_CTRL;
	if (last_state == State_t::MANUAL_CTRL && (state == State_t::AUTO_HOVER || state == State_t::AUTO_TAKEOFF))
	{
		takeoff_land.landed = false; // Always holds
	}
	last_state = state;

	// 如果无人机处于手动控制模式并且没有解锁,则设置takeoff_land.landed为true
	if (state == State_t::MANUAL_CTRL && state_data.current_state.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
	{
		takeoff_land.landed = true;
		return; // No need of other decisions
	}

	// land_detector parameters
	constexpr double POSITION_DEVIATION_C = -0.5; // Constraint 1: target position below real position for POSITION_DEVIATION_C meters.
	constexpr double VELOCITY_THR_C = 0.1;		  // Constraint 2: velocity below VELOCITY_MIN_C m/s.
	constexpr double TIME_KEEP_C = 3.0;			  // Constraint 3: Time(s) the Constraint 1&2 need to keep.

    static rclcpp::Time time_C12_reached; // time_Constraints12_reached
	static bool is_last_C12_satisfy;
	if (takeoff_land.landed)
	{
		time_C12_reached = node_->now();
		is_last_C12_satisfy = false;
	}
	else
	{
		// 检查约束1和约束2是否满足
		bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;
		if (C12_satisfy && !is_last_C12_satisfy)
		{
			time_C12_reached = node_->now();
		}
		else if (C12_satisfy && is_last_C12_satisfy)
		{
			if ((node_->now() - time_C12_reached).seconds() > TIME_KEEP_C) //约束3满足
			{
				takeoff_land.landed = true;
			}
		}

		is_last_C12_satisfy = C12_satisfy;
	}
}

Desired_State_t PX4CtrlFSM::get_hover_des()
{
	Desired_State_t des;
	des.p = hover_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t PX4CtrlFSM::get_cmd_des()
{
	Desired_State_t des;
	des.p = cmd_data.p;
	des.v = cmd_data.v;
	des.a = cmd_data.a;
	des.j = cmd_data.j;
	des.yaw = cmd_data.yaw;
	des.yaw_rate = cmd_data.yaw_rate;

	return des;
}

Desired_State_t PX4CtrlFSM::get_rotor_speed_up_des(const rclcpp::Time now)
{
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).seconds();
	double des_a_z = exp((delta_t - AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0; // Parameters 6.0 and 7.0 are just heuristic values which result in a saticfactory curve.
	if (des_a_z > 0.1)
	{
		RCLCPP_ERROR(node_->get_logger(),"des_a_z > 0.1!, des_a_z=%f", des_a_z);
		des_a_z = 0.0;
	}

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d(0, 0, des_a_z);
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t PX4CtrlFSM::get_takeoff_land_des(const double speed)
{
	rclcpp::Time now = node_->now();
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).seconds() - (speed > 0 ? AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME : 0); // speed > 0 means takeoff
	// takeoff_land.last_set_cmd_time = now;

	// takeoff_land.start_pose(2) += speed * delta_t;

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
	des.v = Eigen::Vector3d(0, 0, speed);
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

void PX4CtrlFSM::set_hov_with_odom()
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(3) = get_yaw_from_quaternion(odom_data.q);

	last_set_hover_pose_time = node_->now();
}

void PX4CtrlFSM::set_hov_with_rc()
{
	rclcpp::Time now = node_->now();
	double delta_t = (now - last_set_hover_pose_time).seconds();
	last_set_hover_pose_time = now;

	hover_pose(0) += rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? 1 : -1);
	hover_pose(1) += rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.roll ? 1 : -1);
	hover_pose(2) += rc_data.ch[2] * param.max_manual_vel * delta_t * (param.rc_reverse.throttle ? 1 : -1);
	hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? 1 : -1);

	if (hover_pose(2) < -0.3)
		hover_pose(2) = -0.3;

	// if (param.print_dbg)
	// {
	// 	static unsigned int count = 0;
	// 	if (count++ % 100 == 0)
	// 	{
	// 		cout << "hover_pose=" << hover_pose.transpose() << endl;
	// 		cout << "ch[0~3]=" << rc_data.ch[0] << " " << rc_data.ch[1] << " " << rc_data.ch[2] << " " << rc_data.ch[3] << endl;
	// 	}
	// }
}

void PX4CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom)
{
	takeoff_land.start_pose.head<3>() = odom_data.p;
	takeoff_land.start_pose(3) = get_yaw_from_quaternion(odom_data.q);

	takeoff_land.toggle_takeoff_land_time = node_->now();
}

bool PX4CtrlFSM::rc_is_received(const rclcpp::Time &now_time)
{
	return (now_time - rc_data.rcv_stamp).seconds() < param.msg_timeout.rc;
}
bool PX4CtrlFSM::cmd_is_received(const rclcpp::Time &now_time)
{
	return (now_time - cmd_data.rcv_stamp).seconds() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const rclcpp::Time &now_time)
{
	return (now_time - odom_data.rcv_stamp).seconds() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::imu_is_received(const rclcpp::Time &now_time)
{
	return (now_time - imu_data.rcv_stamp).seconds() < param.msg_timeout.imu;
}

bool PX4CtrlFSM::bat_is_received(const rclcpp::Time &now_time)
{
	return (now_time - bat_data.rcv_stamp).seconds() < param.msg_timeout.bat;
}

bool PX4CtrlFSM::recv_new_odom()
{
	if (odom_data.recv_new_msg)
	{
		odom_data.recv_new_msg = false;
		return true;
	}

	return false;
}

void PX4CtrlFSM::publish_bodyrate_ctrl(const Controller_Output_t &u, const rclcpp::Time &stamp)
{
	px4_msgs::msg::VehicleAttitudeSetpoint msg;

	// 使用PX4消息格式
	msg.timestamp = stamp.nanoseconds() / 1000; // 转换为微秒

	// VehicleAttitudeSetpoint消息没有roll_body, pitch_body, yaw_body字段
	// 这些字段在PX4中通过其他方式处理
	// msg.roll_body = u.bodyrates.x();
	// msg.pitch_body = u.bodyrates.y();
	// msg.yaw_body = u.bodyrates.z();

	// 设置推力
	msg.thrust_body[0] = 0.0;
	msg.thrust_body[1] = 0.0;
	msg.thrust_body[2] = u.thrust;

	ctrl_FCU_pub->publish(msg);
}

void PX4CtrlFSM::publish_attitude_ctrl(const Controller_Output_t &u, const rclcpp::Time &stamp)
{
	px4_msgs::msg::VehicleAttitudeSetpoint msg;

	// 使用PX4消息格式
	msg.timestamp = stamp.nanoseconds() / 1000; // 转换为微秒

	// 设置四元数姿态
	msg.q_d[0] = u.q.w(); // w
	msg.q_d[1] = u.q.x(); // x
	msg.q_d[2] = u.q.y(); // y
	msg.q_d[3] = u.q.z(); // z

	// 设置推力
	msg.thrust_body[0] = 0.0;
	msg.thrust_body[1] = 0.0;
	msg.thrust_body[2] = u.thrust;

	ctrl_FCU_pub->publish(msg);
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::msg::Odometry &odom_msg)
{
	geometry_msgs::msg::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;

	traj_start_trigger_pub->publish(msg);
}
// 切换飞行模式
bool PX4CtrlFSM::toggle_offboard_mode(bool on_off)
{
    // TODO: 使用VehicleCommand替代mavros服务调用
    // 创建VehicleCommand消息来设置飞行模式
    if (on_off)
    {
        state_data.state_before_offboard = state_data.current_state;
        if (state_data.state_before_offboard.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) // Not allowed
            state_data.state_before_offboard.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL;

        // 使用VehicleCommand设置OFFBOARD模式
        px4_msgs::msg::VehicleCommand cmd;
        cmd.timestamp = node_->get_clock()->now().nanoseconds() / 1000; // 转换为微秒
        cmd.param1 = 1.0f; // 1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        cmd.param2 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
        cmd.param3 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
        cmd.param4 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
        cmd.param5 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
        cmd.param6 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
        cmd.param7 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
        cmd.command = 176; // MAV_CMD_DO_SET_MODE
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.confirmation = 0;
        cmd.from_external = true;
        
        vehicle_command_pub->publish(cmd);
        set_offboard_flag = true;	
        // 等待模式实际切换（最多等待1秒）实际等待并没有用
        // auto start_time = node_->now();
        // while ((node_->now() - start_time).seconds() < 1.0)
        // {
        //     if (on_off && state_data.current_state.mode == "OFFBOARD")
        //     {
        //         RCLCPP_INFO(node_->get_logger(), "Successfully entered OFFBOARD mode");
        //         return true;
        //     }
        //     else if (!on_off && state_data.current_state.mode == state_data.state_before_offboard.mode)
        //     {
        //         RCLCPP_INFO(node_->get_logger(), "Successfully exited OFFBOARD mode");
        //         return true;
        //     }
        //     rclcpp::sleep_for(std::chrono::milliseconds(10));
        //     rclcpp::spin_some(node_);
        // }

        // RCLCPP_INFO(node_->get_logger(), "\033[31mMode change timed out! Current mode: %s\033[0m", 
        //     state_data.current_state.mode.c_str());
        return true;
    }
    else
    {
        // 使用VehicleCommand退出OFFBOARD模式
        px4_msgs::msg::VehicleCommand cmd;
        cmd.timestamp = node_->get_clock()->now().nanoseconds() / 1000; // 转换为微秒
        cmd.param1 = 1.0f; // 1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        cmd.param2 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
        cmd.param3 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
        cmd.param4 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
        cmd.param5 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
        cmd.param6 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
        cmd.param7 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
        cmd.command = 176; // MAV_CMD_DO_SET_MODE
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.confirmation = 0;
        cmd.from_external = true;
        
        vehicle_command_pub->publish(cmd);

        // 等待模式实际切换（最多等待1秒）
        // auto start_time = node_->now();
        // while ((node_->now() - start_time).seconds() < 1.0)
        // {
        //     if (on_off && state_data.current_state.mode == "OFFBOARD")
        //     {
        //         RCLCPP_INFO(node_->get_logger(), "Successfully entered OFFBOARD mode");
        //         return true;
        //     }
        //     else if (!on_off && state_data.current_state.mode == state_data.state_before_offboard.mode)
        //     {
        //         RCLCPP_INFO(node_->get_logger(), "Successfully exited OFFBOARD mode");
        //         return true;
        //     }
        //     rclcpp::sleep_for(std::chrono::milliseconds(50));
        //     rclcpp::spin_some(node_);
        // }

        // RCLCPP_INFO(node_->get_logger(), "\033[31mMode change timed out! Current mode: %s\033[0m", 
        //     state_data.current_state.mode.c_str());
        return true;
    }
}

// 解锁和上锁
bool PX4CtrlFSM::toggle_arm_disarm(bool arm)
{
    // 使用VehicleCommand进行解锁/上锁
    px4_msgs::msg::VehicleCommand cmd;
    cmd.timestamp = node_->get_clock()->now().nanoseconds() / 1000; // 转换为微秒
    cmd.param1 = arm ? 1.0f : 0.0f; // 1 = ARM, 0 = DISARM
    cmd.param2 = 0.0f;
    cmd.param3 = 0.0f;
    cmd.param4 = 0.0f;
    cmd.param5 = 0.0f;
    cmd.param6 = 0.0f;
    cmd.param7 = 0.0f;
    cmd.command = arm ? 400 : 401; // MAV_CMD_COMPONENT_ARM_DISARM
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.confirmation = 0;
    cmd.from_external = true;
    
    vehicle_command_pub->publish(cmd);
    
    return true;
}

// 重启飞控
void PX4CtrlFSM::reboot_FCU()
{
    // 使用VehicleCommand重启飞控
    px4_msgs::msg::VehicleCommand cmd;
    cmd.timestamp = node_->get_clock()->now().nanoseconds() / 1000; // 转换为微秒
    cmd.param1 = 1.0f; // 1 = Reboot autopilot
    cmd.param2 = 0.0f; // 0 = Do nothing for onboard computer
    cmd.param3 = 0.0f;
    cmd.param4 = 0.0f;
    cmd.param5 = 0.0f;
    cmd.param6 = 0.0f;
    cmd.param7 = 0.0f;
    cmd.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.confirmation = 1;
    cmd.from_external = true;
    
    vehicle_command_pub->publish(cmd);
    
    RCLCPP_INFO(node_->get_logger(), "Reboot FCU command sent successfully");
}

