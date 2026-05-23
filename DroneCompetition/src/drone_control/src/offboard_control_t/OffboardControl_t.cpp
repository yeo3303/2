#include "OffboardControl_t.h"
#include "math_t.h"
#include <cmath>
/*
	北360|0偏航角 顺时针          90
		 y+                    |y+
西        x+ 东  <=> ---- x-____|______x+  0 偏航角 逆时针
270       90                   |
							   |y+
	  南180                     -90

 世界坐标系(东北天)    飞机坐标系(base_link)
 -> g2l_location
	aircraft_deg = 90 - world_deg
	aircraft_deg %= 360.0

*/

void OffboardControl::timer_callback(void)
{
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "当前时间：%f", get_cur_time());
	// if(!print_info_)
	// {
		// RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "收到坐标c(%f, %f) h(%f ,%f), flag_servo = %d yaw=%f",
		// _yolo->get_x(YOLO::TARGET_TYPE::CIRCLE), _yolo->get_y(YOLO::TARGET_TYPE::CIRCLE),
		// _yolo->get_x(YOLO::TARGET_TYPE::H), _yolo->get_y(YOLO::TARGET_TYPE::H),
		// _yolo->get_servo_flag(), get_yaw());
	// }
	// RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "当前飞机位置 x: %f y: %f z: %f yaw: %f",
	// 	get_x_pos(), get_y_pos(), get_z_pos(), get_yaw());
	

	// 桶1（1 -31） 2 (2 -32) 3 (-1 -33)
	
	// 检查位置数据的有效性，防止段错误
	if (!debug_mode_ && !print_info_) {
		if (_motors->get_system_status() == Motors::State__system_status::MAV_STATE_UNINIT) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "MAVROS待启动，等待MAVROS创建ROS节点...");
			return;
		} else if (_motors->get_system_status() == Motors::State__system_status::MAV_STATE_BOOT) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "飞控正在启动，等待...");
			return;
		} else if (_motors->get_system_status() == Motors::State__system_status::MAV_STATE_CALIBRATING) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "飞控正在校准，尚未准备好飞行，等待...");
			return;
		} else if (_motors->get_system_status() != Motors::State__system_status::MAV_STATE_STANDBY &&
					_motors->get_system_status() != Motors::State__system_status::MAV_STATE_ACTIVE) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "当前飞机状态 %d %s",
			_motors->get_system_status_uint8_t(), _motors->get_state_name().c_str());
		}
		if (!isfinite(get_x_pos()) || !isfinite(get_y_pos()) || !isfinite(get_z_pos())) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "位置数据无效，等待有效GPS信号(EKF3 IMU0/1 is using GPS)...");
			return;
		}
	}

	if (debug_mode_) { // 调试模式下，强制设置飞机位置
		_inav->position.x() = 0;
		_inav->position.y() = 0;
		_inav->position.z() = 1.2;
	}
	
	float roll, pitch, yaw;
	get_euler(roll, pitch, yaw);

	if (debug_mode_ && _motors->get_system_status() == Motors::State__system_status::MAV_STATE_UNINIT) {
		roll = 0.0f;
		pitch = 0.0f;
	}

	// 测试目标可视化
	if (debug_mode_) {
		bool shot;
		Doshot(0, shot);
		Doland();
	}

	// 调试输出：像素坐标和相机位置
	// std::cout << "相机当前位置: (" << _camera_gimbal->position[0] << ", " << _camera_gimbal->position[1] << ", " << _camera_gimbal->position[2] << ")" << std::endl;
	// std::cout << "相机旋转角度: roll=" << _camera_gimbal->rotation[0] << " pitch=" << _camera_gimbal->rotation[1] << " yaw=" << _camera_gimbal->rotation[2] << std::endl;
	// std::cout << "相机内参: fx=" << _camera_gimbal->fx << " fy=" << _camera_gimbal->fy << " cx=" << _camera_gimbal->cx << " cy=" << _camera_gimbal->cy << std::endl;
	if (_motors->mode == "LAND" && state_machine_.get_current_state() != FlyState::init && state_machine_.get_current_state() != FlyState::takeoff && state_machine_.get_current_state() != FlyState::end && !print_info_)
	{
		RCLCPP_INFO(this->get_logger(), "飞行结束，进入结束状态");
		state_machine_.transition_to(FlyState::end);
		return;
	}

	state_machine_.execute_dynamic_tasks();
	state_machine_.process_states<
		FlyState::init,
		FlyState::takeoff,

		FlyState::end,
		// 
		FlyState::Goto_Setpoint,
		FlyState::Doland,
		FlyState::LandToStart,
		//
		FlyState::Termial_Control,
		FlyState::Print_Info,
		FlyState::Reflush_config,
		FlyState::MYPID

	>();
	// 发布当前状态
	publish_current_state();
	// if (state_machine_.get_current_state() == FlyState::Goto_Setpoint)
	// {
    //     RCLCPP_INFO(get_logger(),"当前GPS位置（EKF融合后）: x: %f y: %f z: %f", get_x_pos(), get_y_pos(), get_z_pos());
	// }
	
}


void OffboardControl::FlyState_init()
{
		// 读取到配置文件中的投弹区和侦查区坐标
		// headingangle_compass为罗盘读数
	// angle为四元数的角度，
	// dx,dy为飞机坐标系下的横纵坐标
	// x,y为global坐标下的，x正指向正东，y正指向正北
		// float tx_shot = dx_shot - 0.5;
		// float ty_shot = dy_shot;
		// float tx_see = dx_see;
		// float ty_see = dy_see;
		// 旋转到飞机坐标系当前机头朝向角度
	float x_shot, y_shot, x_see, y_see;
	rotate_global2stand(dx_shot, dy_shot, tx_shot, ty_shot);
	rotate_global2stand(dx_see, dy_see, tx_see, ty_see);
	RCLCPP_INFO(this->get_logger(), "默认方向下角度：%f", default_yaw);
	RCLCPP_INFO(this->get_logger(), "罗盘方向投弹区起点 x: %f   y: %f    angle: %f", tx_shot, ty_shot, default_yaw);
	RCLCPP_INFO(this->get_logger(), "罗盘方向侦查区起点 x: %f   y: %f    angle: %f", tx_see, ty_see, default_yaw);
	rotate_world2start(tx_shot, ty_shot, x_shot, y_shot);
	rotate_world2start(tx_see, ty_see, x_see, y_see);
	RCLCPP_INFO(this->get_logger(), "飞机起飞方向投弹区起点 x: %f   y: %f    angle: %f", x_shot, y_shot, default_yaw);
	RCLCPP_INFO(this->get_logger(), "飞机起飞方向侦查区起点 x: %f   y: %f    angle: %f", x_see, y_see, default_yaw);
	rotate_world2local(tx_shot, ty_shot, x_shot, y_shot);
	rotate_world2local(tx_see, ty_see, x_see, y_see);
	RCLCPP_INFO(this->get_logger(), "当前朝向投弹区起点 x: %f   y: %f    angle: %f", x_shot, y_shot, default_yaw);
	RCLCPP_INFO(this->get_logger(), "当前朝向侦查区起点 x: %f   y: %f    angle: %f", x_see, y_see, default_yaw);
	 
	// RCLCPP_INFO(this->get_logger(), "开始初始化舵机");
	// 初始化舵机操作 等待舵机初始化
	
	// while (!_servo_controller.client_->wait_for_service(std::chrono::seconds(1)))
	// {
	// 		if (!rclcpp::ok())
	// 		{
	// 				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the Servo service. Exiting.");
	// 				return;
	// 		}
	// 		RCLCPP_INFO(this->get_logger(), "Servo Service not available, waiting again...");
	// }

  // servo_controller(12, 1050);
  // RCLCPP_INFO(this->get_logger(), "结束初始化舵机");

	// rclcpp::sleep_for(1s);


	if (!isfinite(get_x_pos()) || fabs(sqrt(_inav->orientation.w()*_inav->orientation.w() + _inav->orientation.x()*_inav->orientation.x() + _inav->orientation.y()*_inav->orientation.y() + _inav->orientation.z()*_inav->orientation.z()) - 1.0f) > 0.1f)
	{
		// THROTTLE表示节流的意思，以下代码节流时间间隔为 500 毫秒（即 5 秒）．
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "没有获取到位置数据，等待GPS信号...");
		return;
	}
	
	// 飞控的扩展卡尔曼滤波器（EKF3）已经为IMU（惯性测量单元）0和IMU1设置了起点。
	start = {get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()}; 
	// 飞控日志 AP: Field Elevation Set: 0m 设定当前位置的地面高度为0米，这对于高度控制和避免地面碰撞非常重要。
	// start_global = {get_lat(), get_lon(), get_alt()};			
	
	RCLCPP_INFO(this->get_logger(), "初始旋转角: %f", get_yaw());
	_pose_control->set_dt(0.05); // 设置执行周期（用于PID）

	// 重新设置家地址
	if (_motors->get_system_status() != Motors::State__system_status::MAV_STATE_ACTIVE)
		_motors->set_home_position(get_yaw());

	_motors->switch_mode("GUIDED");

	reset_wp_limits();
}

// 发布状态
void OffboardControl::publish_current_state()
{
  // auto message = std_msgs::msg::String();
	auto message = std_msgs::msg::Int32();
  // message.data = "Current state: " + std::to_string(static_cast<int>(state_machine_.current_state_));
	message.data = fly_state_to_int(state_machine_.get_current_state());
  state_publisher_->publish(message);
}


// 指定间隔时间循环执行航点
// halt为高度，way_points为航点集合，description为航点描述
bool OffboardControl::waypoint_goto_next(float halt, vector<Vector2f> &way_points, float time, int *count, const std::string &description)
{
	static std::vector<Vector2f>::size_type surround_cnt = 0; // 修改类型
	int count_n = count == nullptr? surround_cnt : *count;
    float x_temp = 0.0;
	float y_temp = 0.0;
	if(count!=nullptr)
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "(T 1s) w_g_n,counter: %d, time=%lf", *count, waypoint_timer_.elapsed());
	x_temp = way_points[count_n].x();
	y_temp = way_points[count_n].y();
	if ( waypoint_timer_.elapsed() > time || (is_equal(get_x_pos(), x_temp, 0.2f) && is_equal(get_y_pos(), y_temp, 0.2f))) 
	{
		if (static_cast<std::vector<Vector2f>::size_type>(count_n) >= way_points.size())
		{
			RCLCPP_INFO(this->get_logger(), "w_g_n, %s已经全部遍历", description.c_str());
			count == nullptr? surround_cnt = 0 : *count = 0;
			// waypoint_timer_.reset();
			waypoint_timer_.set_start_time_to_default();
			return true;
	    } 
		else 
		{
		count == nullptr? surround_cnt++ : (*count)++;
		RCLCPP_INFO(this->get_logger(), "w_g_n, %s点位%d x: %lf   y: %lf, timeout=%s", description.c_str(), count_n, x_temp, y_temp, (is_equal(get_x_pos(), x_temp, 0.2f) && is_equal(get_y_pos(), y_temp, 0.2f))? "true" : "false");

		send_world_setpoint_command(x_temp, y_temp, halt, 0.0); // 发送世界坐标系下的航点指令
		RCLCPP_INFO(this->get_logger(), "前往下一点");
		waypoint_timer_.reset();
	    }
		
	}
	return false;	
    
}



// OffboardControl.h #define TRAIN_PID
struct PIDDataPoint
{
	double p;
	double i;
	double d;
	double error;
	double time;
} data_point;

void OffboardControl::send_start_setpoint_command(float x, float y, float z, float yaw){
	_pose_control->send_local_setpoint_command(x + get_x_home_pos(), y + get_y_home_pos(), z, default_yaw - yaw);
}

void OffboardControl::send_world_setpoint_command(float x, float y, float z, float yaw){
	// yaw = fmod(M_PI / 2 - yaw + 2 * M_PI, 2 * M_PI);
	// default_yaw M_PI/2 - headingangle_compass 
	_pose_control->send_local_setpoint_command(x, y, z, default_yaw - yaw);
}

bool OffboardControl::local_setpoint_command(float x, float y, float z, float yaw, double accuracy){
	return _pose_control->local_setpoint_command(
		Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
		Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
		accuracy);
}

bool OffboardControl::trajectory_setpoint(float x, float y, float z, float yaw, double accuracy)
{
	RCLCPP_INFO_ONCE(this->get_logger(), "trajectory_setpoint转换后目标位置：%f %f", x, y);
	return _pose_control->trajectory_setpoint(
			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
			Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
			accuracy);
}
// bool OffboardControl::trajectory_setpoint_world(float x, float y, float z, float yaw, PID::Defaults defaults, double accuracy)
// {
// 	return _pose_control->trajectory_setpoint_world(
// 			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
// 			Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
// 			defaults,
// 			accuracy);
// }
bool OffboardControl::trajectory_setpoint_world(float x, float y, float z, float yaw, double accuracy)
{
	return _pose_control->trajectory_setpoint_world(
			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
			Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
			accuracy);
}

bool OffboardControl::publish_setpoint_world(float x, float y, float z, float yaw, double accuracy)
{
	return _pose_control->publish_setpoint_world(
			Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
			Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
			accuracy);
}

void OffboardControl::send_velocity_command(float x, float y, float z, float yaw)
{
	return _pose_control->send_velocity_command(
			Vector4f{x, y, z, yaw});
}
bool OffboardControl::send_velocity_command_with_time(float x, float y, float z, float yaw, double time)
{
	return _pose_control->send_velocity_command_with_time(
			Vector4f{x, y, z, yaw},
			time);
}
bool OffboardControl::trajectory_circle(float a, float b, float height, float dt, float yaw)
{
	return _pose_control->trajectory_circle(
			a,
			b,
			(height - get_z_pos()),
			dt,
			yaw + default_yaw,
			default_yaw);
}
bool OffboardControl::trajectory_generator_world(double speed_factor, std::array<double, 3> q_goal)
{
	return _pose_control->trajectory_generator_world(
			speed_factor,
			q_goal);
}
// bool OffboardControl::trajectory_generator(double speed_factor, std::array<double, 3> q_goal){
// 	rotate_xy(q_goal[0], q_goal[1], default_yaw);
// 	return _pose_control->trajectory_generator_world(
// 		speed_factor,
// 		{q_goal[0]+get_x_pos(), q_goal[1]+get_y_pos(),q_goal[2]+get_z_pos()}
// 	);
// }
bool OffboardControl::trajectory_generator_world_points(double speed_factor, const std::vector<std::array<double, 3>> &data, int data_length, Vector3f max_speed_xy, Vector3f max_accel_xy, float tar_yaw)
{
	static bool first = true;
	static uint16_t data_length_;
	static uint16_t current_waypoint_index = 0;
	static bool sequence_completed = false;
	
	// 只在第一次调用或者序列完成后需要重新开始时初始化
	if (first || (sequence_completed))
	{
		data_length_ = data_length;
		current_waypoint_index = 0;
		sequence_completed = false;
		first = false;
		std::cout << "Initializing waypoint sequence: data_length=" << data_length_ << std::endl;
	}
	
	std::cout << "data.size(): " << data.size() << std::endl;
	std::cout << "data_length: " << data_length_ << std::endl;
	std::cout << "current_waypoint_index: " << current_waypoint_index << std::endl;
	
	// 检查数据有效性，防止越界访问
	if (data.empty() || current_waypoint_index >= data.size() || current_waypoint_index >= data_length_) {
		std::cout << "数据无效或已完成，返回true" << std::endl;
		sequence_completed = true;
		return true;
	}

	std::array<double, 3> q_goal = data[current_waypoint_index];
	double global_x, global_y;
	rotate_global2stand(q_goal[0], q_goal[1], global_x, global_y);
	std::cout << "waypoint " << current_waypoint_index << " q_goal: " << global_x << " " << global_y << " " << q_goal[2] << std::endl;
	
	if (_pose_control->trajectory_generator_world(
				speed_factor,
				{global_x, global_y, q_goal[2]},
				max_speed_xy,
				max_accel_xy,
				static_cast<float>(tar_yaw + default_yaw)
			)
		)
	{
		current_waypoint_index++;
		std::cout << "Waypoint " << (current_waypoint_index - 1) << " completed! Moving to waypoint " << current_waypoint_index << std::endl;
		
		// 检查是否完成所有waypoint
		if (current_waypoint_index >= data.size() || current_waypoint_index >= data_length_)
		{
			std::cout << "All waypoints completed!" << std::endl;
			sequence_completed = true;
			return true;
		}
	}
	
	return false;
}
bool OffboardControl::Doshot(int shot_count, bool &shot_flag) {
    // TODO: 实现你的投弹逻辑
    return false;
}

bool OffboardControl::Doland() {
    // TODO: 实现你的降落逻辑
    return false;
}