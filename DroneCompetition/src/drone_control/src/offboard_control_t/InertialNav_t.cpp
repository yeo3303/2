#include "rclcpp/rclcpp.hpp"

#include "InertialNav_t.h"
#include "math_t.h"

// Vector3f InertialNav::position;
// Vector4f InertialNav::velocity;
// Vector3f InertialNav::gps;
// Quaternionf InertialNav::orientation;
// float InertialNav::altitude;
// Vector3f InertialNav::linear_acceleration;
// Vector3f InertialNav::angular_velocity;
// float InertialNav::rangefinder_height;
// float InertialNav::roll, InertialNav::pitch, InertialNav::yaw; // 欧拉角


// 接收位置数据
void InertialNav::status_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
	position = Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	orientation = Quaternionf(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
	velocity = Vector4f(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z, msg->twist.twist.angular.z);
    // RCLCPP_INFO(node->get_logger(),"position x: %lf", pose_.pose.position.x);
	// std::cout << "position x: " << msg->pose.position.x << std::endl;
	// std::cout << "position y: " << msg->pose.position.y  << std::endl;
	// std::cout << "position z: " << msg->pose.position.z  << std::endl;
	// RCLCPP_INFO(node->get_logger(), "Received yaw: %f", quaternion_to_yaw(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
}
// 接收GPS数据
void InertialNav::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) 
{
	gps = Vector3f(msg->latitude, msg->longitude, msg->altitude);
}
void InertialNav::gps1_raw_callback(const mavros_msgs::msg::GPSRAW::SharedPtr msg)
{
    // 发布GPS1原始数据
    RCLCPP_INFO(node->get_logger(), "GPS1 Raw: lat: %d, lon: %d, alt: %d", msg->lat, msg->lon, msg->alt);
}
void InertialNav::gps2_raw_callback(const mavros_msgs::msg::GPSRAW::SharedPtr msg)
{
    // 发布GPS2原始数据
    RCLCPP_INFO(node->get_logger(), "GPS2 Raw: lat: %d, lon: %d, alt: %d", msg->lat, msg->lon, msg->alt);
}
// 接收高度数据=0
void InertialNav::altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr msg) 
{
	altitude = msg->monotonic;
}
// 接收IMU数据
void InertialNav::imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
	linear_acceleration = Vector3f(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
	angular_velocity = Vector3f(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
}

// 接收激光雷达数据
void InertialNav::rangefinder_callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
	rangefinder_height = msg->range;
}