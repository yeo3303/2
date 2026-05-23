#include <rclcpp/rclcpp.hpp>
#include "OffboardControl_t.h"

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor executor;
	auto node = std::make_shared<OffboardControl>("/mavros/");
	/* 运行节点，并检测退出信号*/
	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}