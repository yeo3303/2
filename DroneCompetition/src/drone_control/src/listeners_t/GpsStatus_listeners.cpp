#include "rclcpp/rclcpp.hpp"
#include "OffboardControl_Base_t.h"

#include <mavros_msgs/msg/gpsraw.hpp> 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/altitude.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "sensor_msgs/msg/range.hpp"

#include "math_t.h"

class GPSStatus : public rclcpp::Node
{
public:
    GPSStatus() : rclcpp::Node("gps_status_node")
    {
        gps1_raw_sub_ = this->create_subscription<mavros_msgs::msg::GPSRAW>(
            "/mavros/gpsstatus/gps1/raw", 10,
            std::bind(&GPSStatus::gps1_raw_callback, this, std::placeholders::_1));
        gps2_raw_sub_ = this->create_subscription<mavros_msgs::msg::GPSRAW>(
            "/mavros/gpsstatus/gps2/raw", 10,
            std::bind(&GPSStatus::gps2_raw_callback, this, std::placeholders::_1));
    }
private:
    rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr gps1_raw_sub_;
    rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr gps2_raw_sub_;

    void gps1_raw_callback(const mavros_msgs::msg::GPSRAW::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "GPS1 Raw: lat: %d, lon: %d, alt: %d", msg->lat, msg->lon, msg->alt);
    }
    void gps2_raw_callback(const mavros_msgs::msg::GPSRAW::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "GPS2 Raw: lat: %d, lon: %d, alt: %d", msg->lat, msg->lon, msg->alt);
    }
};

int main(int argc, char *argv[])
{
	std::cout << "Starting gps_status listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GPSStatus>());

	rclcpp::shutdown();
	return 0;
}
