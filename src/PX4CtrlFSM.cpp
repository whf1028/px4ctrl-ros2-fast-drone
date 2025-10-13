#include "PX4CtrlFSM.h"
#include "FlightLogger.h"

using namespace std;
using namespace uav_utils;

// 构造函数,初始化各个组件
PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, LinearControl &controller_, rclcpp::Node::SharedPtr node) 
    : node_(node),            // ROS2节点
      param(param_),          // 参数配置
      rc_data(node),          // 遥控器数据
      state_data(node),       // 状态数据
      extended_state_data(node), // 扩展状态数据
      odom_data(node),        // 里程计数据
      imu_data(node),         // IMU数据
      cmd_data(node),         // 指令数据
      bat_data(node),         // 电池数据
      takeoff_land_data(node), // 起飞降落数据
      controller(controller_) // 控制器
{
    state = MANUAL_CTRL;      // 初始状态为手动控制
	set_offboard_flag = false;  // 初始化起飞降落标志
	takeoff_land_data.triggered = false;
	FLIGHT_LOG_INFO(FLIGHT_PHASE, "[px4ctrl] NONE --> MANUAL_CTRL(L1)");
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

    // 详细的状态检查日志
    FLIGHT_LOG_INFO(SYSTEM, "状态机处理开始 - 时间戳: %.3f", now_time.seconds());
    FLIGHT_LOG_INFO(SYSTEM, "当前状态: %d", (int)state);
    
    // 检查各种数据是否超时
    bool rc_ok = rc_is_received(now_time);
    bool odom_ok = odom_is_received(now_time);
    bool imu_ok = imu_is_received(now_time);
    bool cmd_ok = cmd_is_received(now_time);
    bool bat_ok = bat_is_received(now_time);
    
    FLIGHT_LOG_INFO(SENSOR, "传感器数据状态 - RC: %s, Odom: %s, IMU: %s, Cmd: %s, Bat: %s", 
                    rc_ok ? "OK" : "TIMEOUT", odom_ok ? "OK" : "TIMEOUT", 
                    imu_ok ? "OK" : "TIMEOUT", cmd_ok ? "OK" : "TIMEOUT", 
                    bat_ok ? "OK" : "TIMEOUT");
    
    if (!rc_ok || !odom_ok || !imu_ok)
    {
        FLIGHT_LOG_WARN(SYSTEM, "传感器数据超时，跳过本次处理");
        return;
    }

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

        FLIGHT_LOG_INFO(FLIGHT_PHASE, 
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
		FLIGHT_LOG_INFO(FLIGHT_PHASE, "处理MANUAL_CTRL状态");
		
		if (rc_data.enter_hover_mode) // 尝试跳转到AUTO_HOVER
		{
			FLIGHT_LOG_INFO(FLIGHT_PHASE, "检测到进入悬停模式指令，准备切换到AUTO_HOVER状态");
			FLIGHT_LOG_INFO(FLIGHT_PHASE, "检测到进入悬停模式指令");
			if(!odom_is_received(now_time))
			{
				// 拒绝AUTO_HOVER(L2),因为没有里程计数据!
				FLIGHT_LOG_ERROR(FLIGHT_PHASE, "Reject AUTO_HOVER(L2). No odom!");
				break;
			}
			if(cmd_is_received(now_time))
			{
				// 拒绝AUTO_HOVER(L2),因为你在进入AUTO_HOVER之前发送了指令,这是不允许的,请立即停止发送指令!
				FLIGHT_LOG_ERROR(FLIGHT_PHASE, "Reject AUTO_HOVER(L2). You are sending commands before toggling into AUTO_HOVER, which is not allowed. Stop sending commands now!");
				break;
			}
			if(odom_data.v.norm() > 3.0)
			{
				//OK
				// 拒绝AUTO_HOVER(L2),因为里程计数据表明无人机速度大于3m/s,这可能表明定位模块出现了问题!
				FLIGHT_LOG_ERROR(FLIGHT_PHASE, "Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
				break;
			}
            // 检查是否需要切换Offboard模式
			bool offboard_switch_result = false;
			if (!set_offboard_flag) {
				offboard_switch_result = toggle_offboard_mode(true);
			} else {
				// 如果已经设置了标志，检查当前是否已经在Offboard模式
				offboard_switch_result = (state_data.current_state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
			}
			
			if(set_offboard_flag || offboard_switch_result)// 切换到offboard模式
			{
				FLIGHT_LOG_INFO(FLIGHT_PHASE, "\033[32m111\033[0m");
				
				if (state_data.current_state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
				{
					FLIGHT_LOG_INFO(FLIGHT_PHASE, "\033[32m222\033[0m");

					// 切换到自动悬停模式
					state = AUTO_HOVER;// 切换到自动悬停模式
					controller.resetThrustMapping();// 重置推力映射参数
					set_hov_with_odom();// 设置悬停位置
					set_offboard_flag = false;
					rc_data.enter_hover_mode = false;
					FLIGHT_LOG_INFO(FLIGHT_PHASE, "\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[0m");
					
					FLIGHT_LOG_INFO(FLIGHT_PHASE, "状态转换: MANUAL_CTRL -> AUTO_HOVER");
					FLIGHT_LOG_DEBUG(CONTROLLER, "控制器推力映射重置完成");
					FLIGHT_LOG_DEBUG(FLIGHT_PHASE, "悬停位置设置完成，基于当前里程计数据");
				}
				else
				{
					FLIGHT_LOG_INFO(FLIGHT_PHASE, "\033[32mIt's OK!Wait for offboard mode...\033[0m");
					break;
				}
			}
			else
			{
				FLIGHT_LOG_INFO(FLIGHT_PHASE, "\033[31mReject AUTO_HOVER. Failed to toggle offboard mode!\033[0m");
				break;
			}
		}

		else if (param.takeoff_land.enable && takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::TAKEOFF) // Try to jump to AUTO_TAKEOFF
		{
			FLIGHT_LOG_INFO(FLIGHT_PHASE, "检测到起飞命令，开始起飞条件检查");
			FLIGHT_LOG_INFO(FLIGHT_PHASE, "🚀 [状态机] 检测到起飞命令 - 开始起飞条件检查");

			if (!odom_is_received(now_time))// 拒绝AUTO_TAKEOFF,因为没有里程计数据!
			{
				takeoff_land_data.triggered = false;
				FLIGHT_LOG_ERROR(FLIGHT_PHASE, "❌ [状态机] 拒绝起飞 - 里程计数据超时!");
				FLIGHT_LOG_ERROR(FLIGHT_PHASE, "起飞条件检查失败 - 里程计数据超时");
				break;
			}
			if (cmd_is_received(now_time))  // 拒绝AUTO_TAKEOFF,因为你正在进入AUTO_TAKEOFF之前发送指令,这是不允许的,请立即停止发送指令!
			{
				takeoff_land_data.triggered = false;
				FLIGHT_LOG_ERROR(FLIGHT_PHASE, "❌ [状态机] 拒绝起飞 - 检测到位置指令，请停止发送指令!");
				FLIGHT_LOG_ERROR(FLIGHT_PHASE, "起飞条件检查失败 - 检测到位置指令");
				break;
			}
			if (odom_data.v.norm() > 0.1)   // 拒绝AUTO_TAKEOFF,因为里程计数据表明无人机速度大于0.1m/s,这是不允许的!
			{
				takeoff_land_data.triggered = false;
				FLIGHT_LOG_ERROR(FLIGHT_PHASE, "❌ [状态机] 拒绝起飞 - 无人机速度过快: %.3fm/s (要求<0.1m/s)", odom_data.v.norm());
				FLIGHT_LOG_ERROR(FLIGHT_PHASE, "起飞条件检查失败 - 无人机速度过快: %.3fm/s", odom_data.v.norm());
				break;
			}
			if (!get_landed())				// 拒绝AUTO_TAKEOFF,因为起飞降落检测器说无人机现在没有降落!
			{
				takeoff_land_data.triggered = false;
				FLIGHT_LOG_ERROR(FLIGHT_PHASE, "❌ [状态机] 拒绝起飞 - 无人机未处于降落状态!");
				FLIGHT_LOG_ERROR(FLIGHT_PHASE, "起飞条件检查失败 - 无人机未处于降落状态");
				break;
			}
			if (rc_is_received(now_time))   // 检查遥控器是否连接
			{
				if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered())
				{
					// 拒绝AUTO_TAKEOFF,如果遥控器没有连接或者没有处于自动悬停模式或指令控制模式,或者摇杆没有居中,请重新起飞!
					FLIGHT_LOG_ERROR(FLIGHT_PHASE, "Reject AUTO_TAKEOFF. If you have your RC connected, keep its switches at \"auto hover\" and \"command control\" states, and all sticks at the center, then takeoff again.");
					while (rclcpp::ok())
					{
						rate4takeoff.sleep();
						// rclcpp::sleep_for(std::chrono::milliseconds(10));
						rclcpp::spin_some(node_);
						if (rc_data.is_hover_mode && rc_data.is_command_mode && rc_data.check_centered())
						{
							// 可以再次起飞
							takeoff_land_data.triggered = false;
							FLIGHT_LOG_INFO(FLIGHT_PHASE, "\033[32m[px4ctrl] OK, you can takeoff again.\033[0m");
							break;
						}
					}
					break;
				}
			}

			// 切换到offboard模式
			FLIGHT_LOG_INFO(FLIGHT_PHASE, "🔄 [状态机] 起飞条件检查通过 - 开始切换到Offboard模式");
			FLIGHT_LOG_INFO(FLIGHT_PHASE, "起飞条件检查通过，开始切换到Offboard模式");
			
			// 检查当前是否已经在Offboard模式
			bool already_in_offboard = (state_data.current_state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
			
			FLIGHT_LOG_INFO(FLIGHT_PHASE, "📊 [状态机] Offboard模式检查 - 当前状态: %d, 目标状态: %d, 已在Offboard: %s", 
					   state_data.current_state.nav_state, 
					   px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD,
					   already_in_offboard ? "true" : "false");
			
			if (already_in_offboard)
			{
				// 已经在Offboard模式，直接进入起飞序列
				FLIGHT_LOG_INFO(FLIGHT_PHASE, "✅ [状态机] 已在Offboard模式 - 开始起飞序列");
				FLIGHT_LOG_INFO(FLIGHT_PHASE, "已在Offboard模式，开始起飞序列");
				
				state = AUTO_TAKEOFF;
				controller.resetThrustMapping();
				set_start_pose_for_takeoff_land(odom_data);	
				
				// 如果启用了自动解锁功能
				// 打印自动解锁功能判断值
				FLIGHT_LOG_INFO(FLIGHT_PHASE, "📊 [状态机] 自动解锁功能检查 - enable_auto_arm: %s", 
						   param.takeoff_land.enable_auto_arm ? "true" : "false");
				FLIGHT_LOG_INFO(FLIGHT_PHASE, "自动解锁功能检查 - enable_auto_arm: %s", 
						   param.takeoff_land.enable_auto_arm ? "true" : "false");
				
				if (param.takeoff_land.enable_auto_arm)
				{
					FLIGHT_LOG_INFO(FLIGHT_PHASE, "🔓 [状态机] 自动解锁功能已启用 - 发送解锁命令");
					toggle_arm_disarm(true);
					FLIGHT_LOG_INFO(FLIGHT_PHASE, "自动解锁功能已启用，发送解锁命令");
				}
				else
				{
					FLIGHT_LOG_INFO(FLIGHT_PHASE, "⚠️ [状态机] 自动解锁功能未启用 - 需要手动解锁");
					FLIGHT_LOG_WARN(FLIGHT_PHASE, "自动解锁功能未启用，需要手动解锁");
				}
				
				takeoff_land.toggle_takeoff_land_time = now_time;
				set_offboard_flag = false;
				takeoff_land_data.triggered = false;
				
				FLIGHT_LOG_INFO(FLIGHT_PHASE, "状态转换: MANUAL_CTRL -> AUTO_TAKEOFF");
				FLIGHT_LOG_DEBUG(CONTROLLER, "控制器推力映射重置完成");
				FLIGHT_LOG_DEBUG(FLIGHT_PHASE, "起飞起始位置设置完成");

				FLIGHT_LOG_INFO(FLIGHT_PHASE, "🚀 [状态机] 状态转换: MANUAL_CTRL -> AUTO_TAKEOFF");
				FLIGHT_LOG_INFO(FLIGHT_PHASE, "📊 [状态机] 起飞参数 - 目标高度: %.2fm, 起飞速度: %.2fm/s", 
						   param.takeoff_land.height, param.takeoff_land.speed);
			}
			else
			{
				// 需要切换到Offboard模式
				FLIGHT_LOG_INFO(FLIGHT_PHASE, "🔄 [状态机] 开始切换到Offboard模式...");
				
				if (toggle_offboard_mode(true))
				{
					set_offboard_flag = true;
					FLIGHT_LOG_INFO(FLIGHT_PHASE, "✅ [状态机] Offboard模式切换成功，等待下次检查");
				}
				else
				{
					FLIGHT_LOG_WARN(FLIGHT_PHASE, "⚠️ [状态机] Offboard模式切换失败，等待下次重试");
				}
				break;
			}
		}

		if (rc_data.toggle_reboot) // Try to reboot. EKF2 based PX4 FCU requires reboot when its state estimator goes wrong.
		{
			if (state_data.current_state.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
			{
				FLIGHT_LOG_ERROR(FLIGHT_PHASE, "Reject reboot! Disarm the drone first!");// 拒绝重启! 先让无人机解锁!
				break;
			}
			rc_data.toggle_reboot = false;
			if (reboot_FCU())
			{
				FLIGHT_LOG_INFO(FLIGHT_PHASE, "✅ [Reboot] 飞控重启命令执行成功");
			}
			else
			{
				FLIGHT_LOG_ERROR(FLIGHT_PHASE, "❌ [Reboot] 飞控重启命令执行失败");
			}
		}

		break;
	}

	case AUTO_HOVER:
	{
		FLIGHT_LOG_INFO(FLIGHT_PHASE, "处理AUTO_HOVER状态");
		
		// 如果遥控器没有进入悬停模式或者没有收到里程计数据
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			FLIGHT_LOG_WARN(FLIGHT_PHASE, "AUTO_HOVER状态检查失败 - 遥控器悬停模式: %s, 里程计数据: %s", 
						   rc_data.is_hover_mode ? "OK" : "FAIL", odom_is_received(now_time) ? "OK" : "TIMEOUT");
			state = MANUAL_CTRL;// 切换到手动控制模式
			toggle_offboard_mode(false);// 关闭offboard模式

			FLIGHT_LOG_WARN(FLIGHT_PHASE, "AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
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
				FLIGHT_LOG_INFO(FLIGHT_PHASE, "\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[0m");
			}
		}
		// 如果起飞降落数据触发并且起飞降落指令为降落
		else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::LAND)
		{
			FLIGHT_LOG_INFO(FLIGHT_PHASE, "🛬 [状态机] 检测到降落命令 - 开始降落序列");
			FLIGHT_LOG_INFO(FLIGHT_PHASE, "检测到降落命令，开始降落序列");
			
			// 如果起飞降落数据触发并且起飞降落指令为降落
			state = AUTO_LAND;// 切换到自动降落模式
			set_start_pose_for_takeoff_land(odom_data);// 设置起飞降落起始位置

			FLIGHT_LOG_INFO(FLIGHT_PHASE, "🛬 [状态机] 状态转换: AUTO_HOVER -> AUTO_LAND");
			FLIGHT_LOG_INFO(FLIGHT_PHASE, "📊 [状态机] 降落参数 - 降落速度: %.2fm/s", param.takeoff_land.speed);
			FLIGHT_LOG_INFO(FLIGHT_PHASE, "状态转换: AUTO_HOVER -> AUTO_LAND");
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
				FLIGHT_LOG_INFO(FLIGHT_PHASE, "\033[32m[px4ctrl] TRIGGER sent, allow user command.\033[0m");
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

			FLIGHT_LOG_WARN(FLIGHT_PHASE, "From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
		}
		else if (!rc_data.is_command_mode || !cmd_is_received(now_time))
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			FLIGHT_LOG_INFO(FLIGHT_PHASE, "From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
		}
		else
		{
			des = get_cmd_des();
		}

		if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::LAND)
		{
			FLIGHT_LOG_ERROR(FLIGHT_PHASE, "Reject AUTO_LAND, which must be triggered in AUTO_HOVER. \
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
			FLIGHT_LOG_INFO(FLIGHT_PHASE, "\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[0m");

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

			FLIGHT_LOG_WARN(FLIGHT_PHASE, "From AUTO_LAND to MANUAL_CTRL(L1)!");
		}
		else if (!rc_data.is_command_mode)
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			FLIGHT_LOG_INFO(FLIGHT_PHASE, "From AUTO_LAND to AUTO_HOVER(L2)!");
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
				FLIGHT_LOG_INFO(FLIGHT_PHASE, "\033[32m[px4ctrl] Wait for abount 10s to let the drone arm.\033[0m");
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
						FLIGHT_LOG_INFO(FLIGHT_PHASE, "\033[32m[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)\033[0m");
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
	FLIGHT_LOG_INFO(CONTROLLER, "开始控制器计算");
	FLIGHT_LOG_INFO(CONTROLLER, "期望位置: [%.3f, %.3f, %.3f]", des.p(0), des.p(1), des.p(2));
	FLIGHT_LOG_INFO(CONTROLLER, "期望速度: [%.3f, %.3f, %.3f]", des.v(0), des.v(1), des.v(2));
	FLIGHT_LOG_INFO(CONTROLLER, "当前位置: [%.3f, %.3f, %.3f]", odom_data.p(0), odom_data.p(1), odom_data.p(2));
	FLIGHT_LOG_INFO(CONTROLLER, "当前速度: [%.3f, %.3f, %.3f]", odom_data.v(0), odom_data.v(1), odom_data.v(2));
	
	if (rotor_low_speed_during_land) // 在自动起飞开始时使用
	{
		FLIGHT_LOG_INFO(CONTROLLER, "使用电机怠速模式");
		motors_idling(imu_data, u);
	}
	else
	{
		FLIGHT_LOG_INFO(CONTROLLER, "执行控制器计算");
		debug_msg = controller.calculateControl(des, odom_data, imu_data, u);
		debug_msg.header.stamp = now_time;
		debug_pub->publish(debug_msg);
		
		FLIGHT_LOG_INFO(CONTROLLER, "控制输出 - 推力: %.3f, 横滚: %.3f, 俯仰: %.3f, 偏航: %.3f", 
						u.thrust, u.q.x(), u.q.y(), u.q.z());
	}

	// STEP4: 发布控制命令到mavros
	FLIGHT_LOG_INFO(CONTROLLER, "发布控制命令到飞控");
	if (param.use_bodyrate_ctrl)
	{
		FLIGHT_LOG_INFO(CONTROLLER, "使用机体角速度控制模式");
		publish_bodyrate_ctrl(u, now_time);
	}
	else
	{
		FLIGHT_LOG_INFO(CONTROLLER, "使用姿态控制模式");
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
		FLIGHT_LOG_ERROR(FLIGHT_PHASE,"des_a_z > 0.1!, des_a_z=%f", des_a_z);
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
	
	// 详细日志记录悬停位置设置
	FLIGHT_LOG_INFO(FLIGHT_PHASE, "基于里程计设置悬停位置 - 位置: [%.3f, %.3f, %.3f], 偏航: %.3f", 
	                hover_pose(0), hover_pose(1), hover_pose(2), hover_pose(3));
	FLIGHT_LOG_INFO(FLIGHT_PHASE, "悬停位置设置完成 - 位置: [%.3f, %.3f, %.3f], 偏航: %.3f", 
	            hover_pose(0), hover_pose(1), hover_pose(2), hover_pose(3));
}

void PX4CtrlFSM::set_hov_with_rc()
{
	rclcpp::Time now = node_->now();
	double delta_t = (now - last_set_hover_pose_time).seconds();
	last_set_hover_pose_time = now;

	// 详细日志记录遥控器悬停位置更新
	static int rc_hover_log_counter = 0;
	if (++rc_hover_log_counter % 50 == 0) { // 每50次记录一次
		FLIGHT_LOG_DEBUG(FLIGHT_PHASE, "遥控器悬停位置更新 - 时间差: %.3f, RC通道: [%.3f, %.3f, %.3f, %.3f]", 
		                 delta_t, rc_data.ch[0], rc_data.ch[1], rc_data.ch[2], rc_data.ch[3]);
		FLIGHT_LOG_DEBUG(FLIGHT_PHASE, "最大手动速度: %.3f, RC反向设置: [%d, %d, %d, %d]", 
		                 param.max_manual_vel, param.rc_reverse.roll, param.rc_reverse.pitch, 
		                 param.rc_reverse.throttle, param.rc_reverse.yaw);
	}

	hover_pose(0) += rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? 1 : -1);
	hover_pose(1) += rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.roll ? 1 : -1);
	hover_pose(2) += rc_data.ch[2] * param.max_manual_vel * delta_t * (param.rc_reverse.throttle ? 1 : -1);
	hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? 1 : -1);

	if (hover_pose(2) < -0.3)
		hover_pose(2) = -0.3;
		
	// 详细日志记录悬停位置更新结果
	if (rc_hover_log_counter % 50 == 0) {
		FLIGHT_LOG_INFO(FLIGHT_PHASE, "悬停位置更新完成 - 位置: [%.3f, %.3f, %.3f], 偏航: %.3f", 
		                 hover_pose(0), hover_pose(1), hover_pose(2), hover_pose(3));
	}

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

void PX4CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t & /* odom */)
{
	takeoff_land.start_pose.head<3>() = odom_data.p;
	takeoff_land.start_pose(3) = get_yaw_from_quaternion(odom_data.q);

	takeoff_land.toggle_takeoff_land_time = node_->now();
	
	// 详细日志记录起飞降落起始位置设置
	FLIGHT_LOG_INFO(FLIGHT_PHASE, "设置起飞降落起始位置 - 位置: [%.3f, %.3f, %.3f], 偏航: %.3f", 
	                takeoff_land.start_pose(0), takeoff_land.start_pose(1), 
	                takeoff_land.start_pose(2), takeoff_land.start_pose(3));
	FLIGHT_LOG_INFO(FLIGHT_PHASE, "起飞降落起始位置设置完成 - 位置: [%.3f, %.3f, %.3f], 偏航: %.3f", 
	            takeoff_land.start_pose(0), takeoff_land.start_pose(1), 
	            takeoff_land.start_pose(2), takeoff_land.start_pose(3));
}

bool PX4CtrlFSM::rc_is_received(const rclcpp::Time &now_time)
{
	double time_diff = (now_time - rc_data.rcv_stamp).seconds();
	bool is_received = time_diff < param.msg_timeout.rc;
	
	// 详细日志记录RC数据接收状态
	static int rc_status_log_counter = 0;
	static bool rc_timeout_warning_shown = false;
	
	if (++rc_status_log_counter % 100 == 0) { // 每100次记录一次
		FLIGHT_LOG_DEBUG(SENSOR, "🔍 [RC状态] 数据检查 - 时间差: %.3fs, 超时阈值: %.3fs, 状态: %s", 
		                 time_diff, param.msg_timeout.rc, is_received ? "OK" : "TIMEOUT");
		
		// 如果RC数据超时，提供详细的诊断信息
		if (!is_received && !rc_timeout_warning_shown) {
			FLIGHT_LOG_ERROR(SENSOR, "❌ [RC状态] RC数据超时警告！");
			FLIGHT_LOG_ERROR(SENSOR, "📊 [RC状态] 诊断信息:");
			FLIGHT_LOG_ERROR(SENSOR, "   - 当前时间: %.3f", now_time.seconds());
			FLIGHT_LOG_ERROR(SENSOR, "   - RC时间戳: %.3f", rc_data.rcv_stamp.seconds());
			FLIGHT_LOG_ERROR(SENSOR, "   - 时间差值: %.3fs (超时阈值: %.3fs)", time_diff, param.msg_timeout.rc);
			FLIGHT_LOG_ERROR(SENSOR, "   - 可能原因: RC订阅器未工作或RC数据源断开");
			FLIGHT_LOG_ERROR(SENSOR, "🚨 [RC状态] 这将导致起飞功能无法正常工作！");
			rc_timeout_warning_shown = true;
		}
		
		// 如果RC数据恢复正常，重置警告标志
		if (is_received && rc_timeout_warning_shown) {
			FLIGHT_LOG_INFO(SENSOR, "✅ [RC状态] RC数据已恢复正常");
			rc_timeout_warning_shown = false;
		}
	}
	
	return is_received;
}

bool PX4CtrlFSM::cmd_is_received(const rclcpp::Time &now_time)
{
	double time_diff = (now_time - cmd_data.rcv_stamp).seconds();
	bool is_received = time_diff < param.msg_timeout.cmd;
	
	// 详细日志记录指令数据接收状态
	static int cmd_status_log_counter = 0;
	if (++cmd_status_log_counter % 100 == 0) { // 每100次记录一次
		FLIGHT_LOG_DEBUG(SENSOR, "指令数据状态检查 - 时间差: %.3f, 超时阈值: %.3f, 状态: %s", 
		                 time_diff, param.msg_timeout.cmd, is_received ? "OK" : "TIMEOUT");
	}
	
	return is_received;
}

bool PX4CtrlFSM::odom_is_received(const rclcpp::Time &now_time)
{
	double time_diff = (now_time - odom_data.rcv_stamp).seconds();
	bool is_received = time_diff < param.msg_timeout.odom;
	
	// 详细日志记录里程计数据接收状态
	static int odom_status_log_counter = 0;
	if (++odom_status_log_counter % 100 == 0) { // 每100次记录一次
		FLIGHT_LOG_DEBUG(SENSOR, "里程计数据状态检查 - 时间差: %.3f, 超时阈值: %.3f, 状态: %s", 
		                 time_diff, param.msg_timeout.odom, is_received ? "OK" : "TIMEOUT");
	}
	
	return is_received;
}

bool PX4CtrlFSM::imu_is_received(const rclcpp::Time &now_time)
{
	double time_diff = (now_time - imu_data.rcv_stamp).seconds();
	bool is_received = time_diff < param.msg_timeout.imu;
	
	// 详细日志记录IMU数据接收状态
	static int imu_status_log_counter = 0;
	if (++imu_status_log_counter % 100 == 0) { // 每100次记录一次
		FLIGHT_LOG_DEBUG(SENSOR, "IMU数据状态检查 - 时间差: %.3f, 超时阈值: %.3f, 状态: %s", 
		                 time_diff, param.msg_timeout.imu, is_received ? "OK" : "TIMEOUT");
	}
	
	return is_received;
}

bool PX4CtrlFSM::bat_is_received(const rclcpp::Time &now_time)
{
	double time_diff = (now_time - bat_data.rcv_stamp).seconds();
	bool is_received = time_diff < param.msg_timeout.bat;
	
	// 详细日志记录电池数据接收状态
	static int bat_status_log_counter = 0;
	if (++bat_status_log_counter % 100 == 0) { // 每100次记录一次
		FLIGHT_LOG_DEBUG(SENSOR, "电池数据状态检查 - 时间差: %.3f, 超时阈值: %.3f, 状态: %s", 
		                 time_diff, param.msg_timeout.bat, is_received ? "OK" : "TIMEOUT");
	}
	
	return is_received;
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
	
	// 详细日志记录机体角速度控制命令发布
	static int bodyrate_log_counter = 0;
	if (++bodyrate_log_counter % 50 == 0) { // 每50次记录一次
		FLIGHT_LOG_INFO(CONTROLLER, "发布机体角速度控制命令 - 推力: %.3f, 时间戳: %.3f", 
		                 u.thrust, stamp.seconds());
		FLIGHT_LOG_DEBUG(FLIGHT_PHASE, "机体角速度控制命令 - 推力: [%.3f, %.3f, %.3f], 时间戳: %lu", 
		             msg.thrust_body[0], msg.thrust_body[1], msg.thrust_body[2], msg.timestamp);
	}
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
	
	// 详细日志记录姿态控制命令发布
	static int attitude_log_counter = 0;
	if (++attitude_log_counter % 50 == 0) { // 每50次记录一次
		FLIGHT_LOG_INFO(CONTROLLER, "发布姿态控制命令 - 四元数: [%.3f, %.3f, %.3f, %.3f], 推力: %.3f", 
		                 u.q.w(), u.q.x(), u.q.y(), u.q.z(), u.thrust);
		FLIGHT_LOG_DEBUG(FLIGHT_PHASE, "姿态控制命令 - 四元数: [%.3f, %.3f, %.3f, %.3f], 推力: [%.3f, %.3f, %.3f], 时间戳: %lu", 
		             msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3], 
		             msg.thrust_body[0], msg.thrust_body[1], msg.thrust_body[2], msg.timestamp);
	}
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::msg::Odometry &odom_msg)
{
	geometry_msgs::msg::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;

	traj_start_trigger_pub->publish(msg);
}
// 切换飞行模式
// bool PX4CtrlFSM::toggle_offboard_mode(bool on_off)
// {
//     // TODO: 使用VehicleCommand替代mavros服务调用
//     // 创建VehicleCommand消息来设置飞行模式
//     if (on_off)
//     {
//         state_data.state_before_offboard = state_data.current_state;
//         if (state_data.state_before_offboard.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) // Not allowed
//             state_data.state_before_offboard.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL;

//         // 使用VehicleCommand设置OFFBOARD模式
//         px4_msgs::msg::VehicleCommand cmd;
//         cmd.timestamp = node_->get_clock()->now().nanoseconds() / 1000; // 转换为微秒
//         cmd.param1 = 1.0f; // 1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
//         cmd.param2 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
//         cmd.param3 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
//         cmd.param4 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
//         cmd.param5 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
//         cmd.param6 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
//         cmd.param7 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
//         cmd.command = 176; // MAV_CMD_DO_SET_MODE
//         cmd.target_system = 1;
//         cmd.target_component = 1;
//         cmd.source_system = 1;
//         cmd.source_component = 1;
//         cmd.confirmation = 0;
//         cmd.from_external = true;
        
//         vehicle_command_pub->publish(cmd);
        
//         // 等待模式实际切换（最多等待2秒）
//         auto start_time = node_->now();
//         while ((node_->now() - start_time).seconds() < 2.0)
//         {
//             if (on_off && state_data.current_state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
//             {
//                 FLIGHT_LOG_INFO(FLIGHT_PHASE, "✅ [Offboard] 成功进入OFFBOARD模式");
//                 return true;
//             }
//             else if (!on_off && state_data.current_state.nav_state == state_data.state_before_offboard.nav_state)
//             {
//                 FLIGHT_LOG_INFO(FLIGHT_PHASE, "✅ [Offboard] 成功退出OFFBOARD模式");
//                 return true;
//             }
//             rclcpp::sleep_for(std::chrono::milliseconds(10));
//             rclcpp::spin_some(node_);
//         }

//         FLIGHT_LOG_WARN(FLIGHT_PHASE, "⚠️ [Offboard] 模式切换超时! 当前导航状态: %d, 目标状态: %d", 
//             state_data.current_state.nav_state, 
//             on_off ? px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD : state_data.state_before_offboard.nav_state);
//         return false;
//     }
//     else
//     {
//         // 使用VehicleCommand退出OFFBOARD模式
//         px4_msgs::msg::VehicleCommand cmd;
//         cmd.timestamp = node_->get_clock()->now().nanoseconds() / 1000; // 转换为微秒
//         cmd.param1 = 1.0f; // 1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
//         cmd.param2 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
//         cmd.param3 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
//         cmd.param4 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
//         cmd.param5 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
//         cmd.param6 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
//         cmd.param7 = 0.0f; // 0 = MAV_MODE_FLAG_AUTO_ENABLED
//         cmd.command = 176; // MAV_CMD_DO_SET_MODE
//         cmd.target_system = 1;
//         cmd.target_component = 1;
//         cmd.source_system = 1;
//         cmd.source_component = 1;
//         cmd.confirmation = 0;
//         cmd.from_external = true;
        
//         vehicle_command_pub->publish(cmd);

//         // 等待模式实际切换（最多等待2秒）
//         auto start_time = node_->now();
//         while ((node_->now() - start_time).seconds() < 2.0)
//         {
//             if (on_off && state_data.current_state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
//             {
//                 FLIGHT_LOG_INFO(FLIGHT_PHASE, "✅ [Offboard] 成功进入OFFBOARD模式");
//                 return true;
//             }
//             else if (!on_off && state_data.current_state.nav_state == state_data.state_before_offboard.nav_state)
//             {
//                 FLIGHT_LOG_INFO(FLIGHT_PHASE, "✅ [Offboard] 成功退出OFFBOARD模式");
//                 return true;
//             }
//             rclcpp::sleep_for(std::chrono::milliseconds(50));
//             rclcpp::spin_some(node_);
//         }

//         FLIGHT_LOG_WARN(FLIGHT_PHASE, "⚠️ [Offboard] 模式切换超时! 当前导航状态: %d, 目标状态: %d", 
//             state_data.current_state.nav_state, 
//             on_off ? px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD : state_data.state_before_offboard.nav_state);
//         return false;
//     }
// }

bool PX4CtrlFSM::toggle_offboard_mode(bool on_off)
{
    if (on_off)
    {
        // 1. 前置条件：持续发送控制消息(使用配置的准备时间)
        auto start_time = node_->now();
        while ((node_->now() - start_time).seconds() < param.fcu_timeout.offboard_prep_time)
        {
            publish_offboard_control_mode();  // 发送OffboardControlMode
            publish_trajectory_setpoint();    // 发送TrajectorySetpoint
            rclcpp::sleep_for(std::chrono::milliseconds(param.fcu_intervals.offboard_prep_ms));
            rclcpp::spin_some(node_);
        }
        
        // 2. 发送VehicleCommand
        px4_msgs::msg::VehicleCommand cmd;
        cmd.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
        cmd.param1 = 1.0f;  // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        cmd.param2 = 6.0f;  // PX4_CUSTOM_MAIN_MODE_OFFBOARD (关键修正!)
        cmd.command = 176;  // MAV_CMD_DO_SET_MODE
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.confirmation = 0;
        cmd.from_external = true;
        
        vehicle_command_pub->publish(cmd);
        
        // 3. 等待模式切换，同时继续发送控制消息
        start_time = node_->now();
        while ((node_->now() - start_time).seconds() < param.fcu_timeout.offboard_mode_switch)
        {
            publish_offboard_control_mode();
            publish_trajectory_setpoint();
            
            if (state_data.current_state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
            {
                FLIGHT_LOG_INFO(FLIGHT_PHASE, "✅ [Offboard] 成功进入OFFBOARD模式");
                return true;
            }
            
            rclcpp::sleep_for(std::chrono::milliseconds(param.fcu_intervals.offboard_check_ms));
            rclcpp::spin_some(node_);
        }
        
        FLIGHT_LOG_WARN(FLIGHT_PHASE, "⚠️ [Offboard] 模式切换超时!");
        return false;
    }
    else
    {
        // ========== 退出OFFBOARD模式 - 简化版本 ==========
        
        // 1. 发送VehicleCommand退出OFFBOARD模式
        px4_msgs::msg::VehicleCommand cmd;
        cmd.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
        
        // 根据之前保存的状态设置目标模式
        uint8_t target_nav_state = state_data.state_before_offboard.nav_state;
        
        // 简化的模式参数设置
        if (target_nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL)
        {
            cmd.param1 = 0.0f;  // 不使用自定义模式
            cmd.param2 = 0.0f;
        }
        else
        {
            cmd.param1 = 1.0f;  // 使用自定义模式
            cmd.param2 = 0.0f;  // 让PX4自动选择合适的主模式
        }
        
        cmd.command = 176;  // MAV_CMD_DO_SET_MODE
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.confirmation = 0;
        cmd.from_external = true;
        
        vehicle_command_pub->publish(cmd);
        
        // 2. 等待模式切换（简化等待逻辑）
        auto start_time = node_->now();
        while ((node_->now() - start_time).seconds() < param.fcu_timeout.offboard_mode_switch)
        {
            // 只要不在OFFBOARD模式就算成功
            if (state_data.current_state.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
            {
                FLIGHT_LOG_INFO(FLIGHT_PHASE, "✅ [Offboard] 成功退出OFFBOARD模式，当前状态: %d", 
                    state_data.current_state.nav_state);
                return true;
            }
            
            rclcpp::sleep_for(std::chrono::milliseconds(param.fcu_intervals.offboard_check_ms));
            rclcpp::spin_some(node_);
        }
        
        // 3. 超时处理
        FLIGHT_LOG_WARN(FLIGHT_PHASE, "⚠️ [Offboard] 退出模式切换超时! 当前导航状态: %d", 
            state_data.current_state.nav_state);
        return false;
    }
}

// 需要添加的辅助函数
void PX4CtrlFSM::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.position = false;       // ❌ 禁用位置控制
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = true;  // ✅ 启用姿态控制
    msg.body_rate = false;
    
    offboard_control_mode_pub->publish(msg);
}

void PX4CtrlFSM::publish_trajectory_setpoint()
{
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    
    // 使用当前位置作为设定点
    msg.position[0] = odom_data.p(0);   // North
    msg.position[1] = odom_data.p(1);   // East  
    msg.position[2] = -odom_data.p(2);  // Down (ENU->NED转换)
    msg.yaw = get_yaw_from_quaternion(odom_data.q);
    
    // 其他字段设为NaN表示不控制
    msg.velocity[0] = std::numeric_limits<float>::quiet_NaN();
    msg.velocity[1] = std::numeric_limits<float>::quiet_NaN();
    msg.velocity[2] = std::numeric_limits<float>::quiet_NaN();
    
    trajectory_setpoint_pub->publish(msg);
}

// 解锁和上锁
bool PX4CtrlFSM::toggle_arm_disarm(bool arm)
{
    FLIGHT_LOG_INFO(FLIGHT_PHASE, "🔄 [Arm/Disarm] 开始执行%s操作", arm ? "解锁" : "上锁");
    
    // 1. 发送VehicleCommand进行解锁/上锁
    px4_msgs::msg::VehicleCommand cmd;
    cmd.timestamp = node_->get_clock()->now().nanoseconds() / 1000; // 转换为微秒
    cmd.param1 = arm ? 1.0f : 0.0f; // 1 = ARM, 0 = DISARM
    cmd.param2 = 0.0f; // 0 = 不使用强制解锁
    // param3-7 对于ARM/DISARM命令不使用，保持默认值0
    cmd.command = arm ? 400 : 401; // MAV_CMD_COMPONENT_ARM_DISARM
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.confirmation = 0;
    cmd.from_external = true;
    
    vehicle_command_pub->publish(cmd);
    
    // 2. 等待状态切换，使用配置的超时时间
    auto start_time = node_->now();
    while ((node_->now() - start_time).seconds() < param.fcu_timeout.arm_disarm_operation)
    {
        // 检查目标状态是否已达成
        bool target_armed_state = (state_data.current_state.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
        
        if (arm && target_armed_state)
        {
            FLIGHT_LOG_INFO(FLIGHT_PHASE, "✅ [Arm/Disarm] 成功解锁");
            return true;
        }
        else if (!arm && !target_armed_state)
        {
            FLIGHT_LOG_INFO(FLIGHT_PHASE, "✅ [Arm/Disarm] 成功上锁");
            return true;
        }
        
        // 使用配置的检查间隔
        rclcpp::sleep_for(std::chrono::milliseconds(param.fcu_intervals.arm_disarm_check_ms));
        rclcpp::spin_some(node_);
    }
    
    // 3. 超时处理
    FLIGHT_LOG_WARN(FLIGHT_PHASE, "⚠️ [Arm/Disarm] %s操作超时! 当前解锁状态: %d, 目标状态: %s", 
                    arm ? "解锁" : "上锁", 
                    state_data.current_state.arming_state,
                    arm ? "ARMED" : "DISARMED");
    return false;
}

// 重启飞控
bool PX4CtrlFSM::reboot_FCU()
{
    FLIGHT_LOG_INFO(FLIGHT_PHASE, "🔄 [Reboot] 开始重启飞控...");
    
    // 1. 检查前置条件：确保无人机已上锁
    if (state_data.current_state.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
    {
        FLIGHT_LOG_ERROR(FLIGHT_PHASE, "❌ [Reboot] 拒绝重启! 无人机仍处于解锁状态，请先上锁!");
        return false;
    }
    
    // 2. 发送VehicleCommand重启飞控
    px4_msgs::msg::VehicleCommand cmd;
    cmd.timestamp = node_->get_clock()->now().nanoseconds() / 1000; // 转换为微秒
    cmd.param1 = 1.0f; // 1 = Reboot autopilot
    cmd.param2 = 0.0f; // 0 = Do nothing for onboard computer
    // param3-7 对于重启命令不使用，保持默认值0
    cmd.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.confirmation = 1; // 重启命令需要确认
    cmd.from_external = true;
    
    vehicle_command_pub->publish(cmd);
    
    FLIGHT_LOG_INFO(FLIGHT_PHASE, "✅ [Reboot] 重启命令已发送");
    
    // 3. 等待飞控重启（通过检查连接状态变化）
    auto start_time = node_->now();
    bool connection_lost = false;
    
    while ((node_->now() - start_time).seconds() < param.fcu_timeout.reboot_operation)
    {
        // 检查USB连接状态
        if (!state_data.current_state.usb_connected)
        {
            if (!connection_lost)
            {
                FLIGHT_LOG_INFO(FLIGHT_PHASE, "📡 [Reboot] 检测到USB连接断开，飞控开始重启...");
                connection_lost = true;
            }
        }
        else if (connection_lost)
        {
            // 连接恢复，说明重启完成
            FLIGHT_LOG_INFO(FLIGHT_PHASE, "✅ [Reboot] USB连接已恢复，飞控重启完成");
            return true;
        }
        
        rclcpp::sleep_for(std::chrono::milliseconds(param.fcu_intervals.reboot_check_ms));
        rclcpp::spin_some(node_);
    }
    
    // 4. 超时处理
    if (connection_lost)
    {
        FLIGHT_LOG_WARN(FLIGHT_PHASE, "⚠️ [Reboot] 飞控重启超时，但连接已断开，可能正在重启中...");
        return true; // 连接已断开，认为重启命令已生效
    }
    else
    {
        FLIGHT_LOG_WARN(FLIGHT_PHASE, "⚠️ [Reboot] 重启命令可能未生效，USB连接未断开");
        return false;
    }
}

