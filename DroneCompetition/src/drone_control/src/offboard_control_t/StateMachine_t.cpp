#include "StateMachine_t.h"
#include "OffboardControl_t.h"

// 具体状态处理实现
template<>
void StateMachine::handle_state<FlyState::init>() {
	if (current_state_ == FlyState::init) {
		owner_->FlyState_init();
		RCLCPP_INFO_ONCE(owner_->get_logger(), "初始化完成");
		if (owner_->debug_mode_) {
			RCLCPP_INFO_ONCE(owner_->get_logger(), "测试模式下, 不进行起飞");
			transition_to(FlyState::Goto_Setpoint);
			return;
		}
		transition_to(FlyState::takeoff);
	}
}

template<>
void StateMachine::handle_state<FlyState::takeoff>() {
	if (current_state_ == FlyState::takeoff){
		RCLCPP_INFO_ONCE(owner_->get_logger(), "开始起飞");

		if (owner_->_motors->takeoff(owner_->get_z_pos(), 2.0, owner_->get_yaw())) {
				RCLCPP_INFO_ONCE(owner_->get_logger(), "起飞成功");
				transition_to(FlyState::Goto_Setpoint);
		} else {
				// RCLCPP_INFO(owner_->get_logger(), "起飞失败");
		}
	}
}

template<>
void StateMachine::handle_state<FlyState::end>() {
	if (current_state_ == FlyState::end) {
		RCLCPP_INFO_ONCE(owner_->get_logger(), "任务开始时间: %f 秒", owner_->get_start_time());
		RCLCPP_INFO_ONCE(owner_->get_logger(), "任务结束, 运行时间: %f 秒", owner_->get_cur_time());
		RCLCPP_INFO_ONCE(owner_->get_logger(), "任务结束, 任务运行时间: %f 秒", owner_->get_cur_time() - owner_->get_start_time());
		if (owner_->state_timer_.elapsed() < 3) {
			return; // 等待 3 秒后结束
		}
		// 如果需要，可以在这里添加清理或退出逻辑
		rclcpp::shutdown();  // 停止 ROS 2 节点
	}
}

template<>
void StateMachine::handle_state<FlyState::Goto_Setpoint>() 
{
	if (current_state_ == FlyState::Goto_Setpoint) 
	{
		static int counter = 0; // 航点计数器
		// RCLCPP_INFO(owner_->get_logger(),"当前GPS位置（EKF融合后）: x: %f y: %f z: %f", owner_->get_x_pos(), owner_->get_y_pos(), owner_->get_z_pos());
		if (owner_->waypoint_goto_next( 
			owner_->see_halt, owner_->set_points, 7 , &counter, "航点飞行"))
		{
			RCLCPP_INFO_ONCE(owner_->get_logger(), "飞行完毕");
			counter = 0;
			// owner_->_motors->switch_mode("RTL");
			// rclcpp::sleep_for(std::chrono::seconds(17));
			// owner_->_motors->switch_mode("GUIDED");
			transition_to(FlyState::LandToStart);
		}
	}
	return;
}

template<>
void StateMachine::handle_state<FlyState::Doland>() {
	if (current_state_ == FlyState::Doland) {
		static enum class DolandState {
			doland_init, // 降落初始化
			doland_wait, // 等待降落
			doland_landing, // 降落中
			doland_end // 降落结束
		} doland_state = DolandState::doland_init; // 降落状态
		if (owner_->is_first_run_) {
			doland_state = DolandState::doland_init; // 重置降落状态
			owner_->is_first_run_ = false; // 重置第一次运行标志
		}
		while (true){
			switch (doland_state) {
			case DolandState::doland_init: // 降落初始化
				RCLCPP_INFO(owner_->get_logger(), "开始降落");
				owner_->_motors->switch_mode("RTL");
				doland_state = DolandState::doland_wait; // 切换到等待降落状态
				continue; // 继续执行下一次循环;
			case DolandState::doland_wait: // 等待降落
				if (owner_->state_timer_.elapsed() > 5) { // 如果等待超过18秒
					RCLCPP_INFO(owner_->get_logger(), "等待降落超过5秒，开始降落");
					owner_->_motors->switch_mode("GUIDED");
					doland_state = DolandState::doland_landing; // 切换到降落中状态
					continue; // 继续执行下一次循环;
				} else {
					RCLCPP_INFO_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 3000, "(THROTTLE 3s)等待降落中...%f", owner_->state_timer_.elapsed());
				}
				break;
			case DolandState::doland_landing: // 降落中
				if(owner_->Doland()){
					doland_state = DolandState::doland_end; // 切换到降落结束状态
					continue; // 继续执行下一次循环;
				}
				break; // 继续执行下一次循环;
			case DolandState::doland_end: // 降落结束
				RCLCPP_INFO(owner_->get_logger(), "降落完成");
				owner_->_motors->switch_mode("LAND");
				transition_to(FlyState::end);
				doland_state = DolandState::doland_init; // 重置降落状态
				break; // 结束函数
			default:
				break;
			}
			break;
		}
	}
	return; // 结束函数
}

template<>
void StateMachine::handle_state<FlyState::LandToStart>() {
	if (current_state_ == FlyState::LandToStart) {
		static enum class LandToStartState {
			land_to_start_init, // 降落初始化
			land_to_start_wait, // 等待降落
			land_to_start_end // 降落结束
		} land_to_start_state = LandToStartState::land_to_start_init; // 降落状态
		if (owner_->is_first_run_) {
			land_to_start_state = LandToStartState::land_to_start_init; // 重置降落状态
			owner_->is_first_run_ = false; // 重置第一次运行标志
		}
		while (true){
			switch (land_to_start_state) {
			case LandToStartState::land_to_start_init: // 降落初始化
				RCLCPP_INFO(owner_->get_logger(), "开始降落");
				owner_->_motors->switch_mode("GUIDED");
				// owner_->_motors->switch_mode("RTL");
				land_to_start_state = LandToStartState::land_to_start_wait; // 切换到等待降落状态
				continue; // 继续执行下一次循环;
			case LandToStartState::land_to_start_wait: // 等待降落
				owner_->send_world_setpoint_command(
					0, 0, 2, 0
				);
				if (owner_->state_timer_.elapsed() > 10 || owner_->_motors->get_system_status() == Motors::State__system_status::MAV_STATE_STANDBY) { // 如果等待超过10秒
					RCLCPP_INFO(owner_->get_logger(), "等待降落超过10秒，开始降落");
					// owner_->_motors->switch_mode("GUIDED");
					land_to_start_state = LandToStartState::land_to_start_end; // 切换到降落中状态
					continue; // 继续执行下一次循环;
				} else {
					RCLCPP_INFO_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 3000, "(THROTTLE 3s)等待降落中...%f", owner_->state_timer_.elapsed());
				}
				break;
			case LandToStartState::land_to_start_end: // 降落结束
				RCLCPP_INFO(owner_->get_logger(), "降落完成");
				owner_->_motors->switch_mode("LAND");
				transition_to(FlyState::end);
				land_to_start_state = LandToStartState::land_to_start_init; // 重置降落状态
				break; // 结束函数
			default:
				break;
			}
			break;
		}
	}
	return; // 结束函数
}

template<>
void StateMachine::handle_state<FlyState::Print_Info>() {
	if (current_state_ == FlyState::Print_Info
	) {
		static unsigned short print_count = 0;
		print_count++; // 先递增
		print_count = print_count % 3; // 然后取模
		if (print_count != 0) {
			return; // 每10次打印一次
		}
		std::stringstream ss;
		// 打印当前状态信息
		ss << "--------timer_callback----------" << std::endl;
		ss << "px:  " << std::setw(10) << owner_->get_x_pos() << ", py: " << std::setw(10) << owner_->get_y_pos() << ", pz: " << std::setw(10) << owner_->get_z_pos() << std::endl;
		ss << "vx:  " << std::setw(10) << owner_->get_x_vel() << ", vy: " << std::setw(10) << owner_->get_y_vel() << ", vz: " << std::setw(10) << owner_->get_z_vel() << std::endl;
		// ss << "yaw: " << owner_->get_yaw() << std::endl;
		// ss << "yaw_e: " << owner_->get_yaw_eigen() << std::endl;
		ss << "yaw_vel: " << owner_->get_yaw_vel() << std::endl;
		float roll, pitch, yaw;
		owner_->get_euler(roll, pitch, yaw);
		ss << "roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << std::endl;
		ss << "lat: " << owner_->get_lat() << ", lon: " << owner_->get_lon() << ", alt: " << owner_->get_alt() << std::endl;
		ss << "rangefinder_distance:  " << owner_->get_rangefinder_distance() << std::endl;
		ss << "armed:     " << owner_->get_armed() << std::endl;
		ss << "connected: " << owner_->get_connected() << std::endl;
		ss << "guided:	" << owner_->get_guided() << std::endl;
		ss << "mode:	  " << owner_->get_mode() << std::endl;
		ss << "system_status:  " << owner_->get_system_status() << std::endl;
		ss << "x_home_pos:     " << owner_->get_x_home_pos() << ", y_home_pos: " << owner_->get_y_home_pos() << ", z_home_pos: " << owner_->get_z_home_pos() << std::endl;
		ss << "running_time: " << owner_->get_cur_time() << std::endl;
		RCLCPP_INFO_STREAM(owner_->get_logger(), ss.str());
		ss.str(""); // 清空字符串流
		ss.clear(); // 清除状态标志
	}
}

template<>
 void StateMachine::handle_state<FlyState::MYPID>() {
	if (current_state_ == FlyState::MYPID
	) {
		owner_->mypid.readPIDParameters("pos_config.yaml","mypid");
		owner_->dx_shot = owner_->mypid.read_goal("OffboardControl.yaml","dx_shot");
		owner_->dy_shot = owner_->mypid.read_goal("OffboardControl.yaml","dy_shot");
		owner_->shot_halt = owner_->mypid.read_goal("OffboardControl.yaml","shot_halt");
		printf("已进入MYPID状态,当前kp ki kd参数分别为：%lf %lf %lf,输出限制为： %lf,积分限制为： %lf\n",owner_->mypid.kp_,owner_->mypid.ki_,owner_->mypid.kd_,owner_->mypid.output_limit_,owner_->mypid.integral_limit);
 
		owner_->mypid.velocity_x = owner_->get_x_vel();
		 owner_->mypid.velocity_y = owner_->get_y_vel();
		owner_->mypid.velocity_z = owner_->get_z_vel();
 
		printf("当前速度分别为：vx: %lf vy: %lf vz:%lf\n",owner_->mypid.velocity_x,owner_->mypid.velocity_y,owner_->mypid.velocity_z);
		printf("当前位置为（ %lf , %lf , %lf ）\n",owner_->get_x_pos(),owner_->get_y_pos(),owner_->get_z_pos());
		printf("目标位置为 （ %lf , %lf , %lf ）\n",owner_->dx_shot,owner_->dy_shot,owner_->shot_halt);
		printf("PID输出分别为：（ %lf , %lf ,%lf）\n",owner_->mypid.compute(owner_->dx_shot,owner_->get_x_pos(),0.01),owner_->mypid.compute(owner_->dy_shot,owner_->get_y_pos(),0.01),
							   owner_->mypid.compute(owner_->shot_halt,owner_->get_z_pos(),0.01));
		
		owner_->mypid.Mypid(owner_->dx_shot,owner_->dy_shot,owner_->shot_halt,owner_->get_x_pos(),owner_->get_y_pos(),owner_->get_z_pos(),0.01);
		owner_->send_velocity_command(owner_->mypid.velocity_x,owner_->mypid.velocity_y,owner_->mypid.velocity_z,0);
		//transition_to(FlyState::end);
	}
 }


template<>
void StateMachine::handle_state<FlyState::Reflush_config>() {
	if (current_state_ == FlyState::Reflush_config) {
		RCLCPP_INFO_ONCE(owner_->get_logger(), "开始刷新配置");
		// 读取配置文件
		owner_->read_configs("OffboardControl.yaml");
		owner_->_pose_control->pid_x_defaults = PID::readPIDParameters("pos_config.yaml","pos_x");
		owner_->_pose_control->pid_y_defaults = PID::readPIDParameters("pos_config.yaml","pos_y");
		owner_->_pose_control->pid_z_defaults = PID::readPIDParameters("pos_config.yaml","pos_z");
		owner_->_pose_control->pid_yaw_defaults = PID::readPIDParameters("pos_config.yaml","pos_yaw");
		owner_->_pose_control->pid_px_defaults = PID::readPIDParameters("pos_config.yaml","pos_px");
		owner_->_pose_control->pid_py_defaults = PID::readPIDParameters("pos_config.yaml","pos_py");
		owner_->_pose_control->pid_pz_defaults = PID::readPIDParameters("pos_config.yaml","pos_pz");
		owner_->_pose_control->pid_vx_defaults = PID::readPIDParameters("pos_config.yaml","pos_vx");
		owner_->_pose_control->pid_vy_defaults = PID::readPIDParameters("pos_config.yaml","pos_vy");
		owner_->_pose_control->pid_vz_defaults = PID::readPIDParameters("pos_config.yaml","pos_vz");
		owner_->_pose_control->limit_defaults = owner_->_pose_control->readLimits("pos_config.yaml","limits");
		// 重新设置PID参数
		owner_->_pose_control->reset_pid();
		owner_->_pose_control->set_limits(owner_->_pose_control->limit_defaults);
		owner_->reset_wp_limits(); // 重置航点速度限制

		RCLCPP_INFO_ONCE(owner_->get_logger(), "配置刷新完成");
		transition_to(previous_state_); // 切换回上一个状态
	}
}

// 始终开启
template<>
void StateMachine::handle_state<FlyState::Termial_Control>() {
  char key = 0;
	static std::string input = "";

  if (_kbhit()) // 检查是否有按键输入
	{
		key = _getch(); // 获取按键输入
		// 特殊状态处理（起飞解锁前）
		if (owner_->_motors->state_ == (Motors::State::wait_for_takeoff_command)){
			if (key == '\n') // 检查是否按下回车键
			{
				owner_->_motors->takeoff_command = true; // 设置起飞命令
			}
			else if (key == 'q') // 检查是否按下q键
			{
				RCLCPP_INFO(owner_->get_logger(), "退出程序");
				transition_to(FlyState::end); // 切换到结束状态
			}
			else if (key != 0)
			{
				RCLCPP_INFO(owner_->get_logger(), "无效输入，请按回车键解锁无人机或按q键退出程序");
			}
			return;
		}
	// 处理多字符输入
	if (key == '\n' || key == '\r') { // 按下回车，尝试解析命令
	  std::string upperInput = input;
	  std::transform(upperInput.begin(), upperInput.end(), upperInput.begin(), ::toupper);
	  auto it = FlyStateMap.find(upperInput);
	  if (it != FlyStateMap.end()) {
		transition_to(it->second);
		RCLCPP_INFO(owner_->get_logger(), "切换到状态: %s", upperInput.c_str());
	  } else {
		RCLCPP_INFO(owner_->get_logger(), "无效指令: %s", input.c_str());
	  }
	  input.clear();
	} else if (key == '\b' && !input.empty()) { // 处理退格
	  input.pop_back();
		} else if (key == '\t') { // Tab键自动补全
			std::string upperInput = input;
			std::transform(upperInput.begin(), upperInput.end(), upperInput.begin(), ::toupper);
			std::vector<std::string> candidates;
			for (const auto& kv : FlyStateMap) {
					if (kv.first.find(upperInput) == 0) { // 前缀匹配
							candidates.push_back(kv.first);
					}
			}
			if (candidates.size() == 1) {
					input = candidates[0];
					RCLCPP_INFO(owner_->get_logger(), "自动补全: %s", input.c_str());
			} else if (candidates.size() > 1) {
					std::string msg = "可选项: ";
					for (const auto& s : candidates) msg += s + " ";
					RCLCPP_INFO(owner_->get_logger(), "%s", msg.c_str());
			}
	} else if (key != 0) {
	  input += key; // 将按键添加到输入字符串中
	}
	}
}

// transition_to函数实现
void StateMachine::transition_to(FlyState new_state) {
	RCLCPP_INFO(owner_->get_logger(), "状态转换: %d -> %d", 
						 static_cast<int>(current_state_), 
						 static_cast<int>(new_state));

	owner_->waypoint_timer_.reset(); // 重置航点计时器
	owner_->state_timer_.reset(); // 重置状态计时器
	// owner_->reset_wp_limits();
	owner_->is_first_run_ = true; // 重置第一次运行标志
	if (new_state == current_state_) {
		RCLCPP_INFO(owner_->get_logger(), "状态未改变，保持当前状态: %d", static_cast<int>(current_state_));
		return;
	}
	previous_state_ = current_state_;
	current_state_ = new_state;
}