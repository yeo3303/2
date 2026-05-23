#pragma once
#ifndef OFFBOARD_CONTROL_BASE_H
#define OFFBOARD_CONTROL_BASE_H

#include "rclcpp/rclcpp.hpp"
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <geometry_msgs/msg/twist_stamped.hpp>
// #include <mavros_msgs/msg/altitude.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
// #include <mavros_msgs/msg/home_position.hpp>
// #include <mavros_msgs/msg/state.hpp>
#include "math_t.h"

#ifndef PAL_STATISTIC_VISIBILITY
#define PAL_STATISTIC_VISIBILITY 0 // 是否开启统计可视化
#endif

// #if PAL_STATISTIC_VISIBILITY
// #include <pal_statistics_msgs/msg/statistics.hpp>
// #include <pal_statistics_msgs/msg/statistic.hpp>
// #endif

#define DEFAULT_YAW (0 * M_PI_2) // default yaw for position control

#define DEFAULT_POS INFINITY

// #define PID_P

// #define TRAIN_PID

class OffboardControl_Base : public rclcpp::Node
{
public:

	OffboardControl_Base(std::string ardupilot_namespace) : Node("Gps_Test_Node"),
		mode_switch_client_{this->create_client<mavros_msgs::srv::SetMode>(ardupilot_namespace + "set_mode")}
	{
		RCLCPP_INFO(this->get_logger(), "Starting GPS Test Node...");

		// Declare and get parameters
		std::stringstream ss;
		ss << "--ros-args -p";
		this->declare_parameter("sim_mode", false);
		this->get_parameter("sim_mode", sim_mode_);
		ss << " sim_mode:=" << (sim_mode_ ? "true" : "false");
		this->declare_parameter("debug_mode", false);
		this->get_parameter("debug_mode", debug_mode_);
		ss << " debug_mode:=" << (debug_mode_ ? "true" : "false");
		this->declare_parameter("print_info", false);
		this->get_parameter("print_info", print_info_);
		ss << " print_info:=" << (print_info_ ? "true" : "false");
		this->declare_parameter("fast_mode", false);
		this->get_parameter("fast_mode", fast_mode_);
		ss << " fast_mode:=" << (fast_mode_ ? "true" : "false");
		RCLCPP_INFO_STREAM(this->get_logger(), ss.str());



	}

	// 获取当前时间
	double get_cur_time() {
		auto now = this->get_clock()->now();
		return now.seconds() - timestamp_init;  // 直接计算时间差并转为秒
	}
	double get_start_time() {
		return start_time;  // 返回开始时间
	}
	void set_start_time(double time) {
		start_time = time;  // 设置开始时间
	}

	bool sim_mode_ = false; // 是否为仿真模式
	bool debug_mode_ = false; // 是否验证状态
	bool print_info_ = false; // 是否打印信息
	bool fast_mode_ = false; // 是否快速模式
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_switch_client_;
	static Vector4f start;

protected:
	double timestamp_init = 0;
	double start_time;

};
#endif // OFFBOARDCONTROL_BASE_H