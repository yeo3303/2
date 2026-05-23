cmake_minimum_required(VERSION 3.5)
project(drone_control VERSION 1.0)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(trajectory_msgs REQUIRED)
#sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.22/Modules/
#find_package(GeographicLib REQUIRED)
#
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Curses REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pal_statistics_msgs QUIET)
if(pal_statistics_msgs_FOUND)
	message(STATUS "PAL Statistics support enabled")
    add_definitions(-DPAL_STATISTIC_VISIBILITY)
    set(PAL_STATISTICS_DEPS pal_statistics_msgs)
else()
    message(WARNING "PAL Statistics requested but pal_statistics_msgs not found")
    set(PAL_STATISTICS_DEPS "")
endif()
#################
# Setup targets #
#################

include_directories(include)

# Add frame_transforms lib
add_library(frame_transforms_t SHARED src/lib/frame_transforms_t.cpp)
ament_target_dependencies(frame_transforms_t Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms_t PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


# listeners/pose_subscriber
add_executable(pose_subscriber_t src/listeners_t/pose_subscriber_t.cpp)
ament_target_dependencies(pose_subscriber_t rclcpp geometry_msgs nav_msgs)
install(TARGETS pose_subscriber_t DESTINATION lib/${PROJECT_NAME})

# # listeners/yolo_subscriber
# add_executable(yolo_subscriber src/listeners/yolo_subscriber.cpp)
# ament_target_dependencies(yolo_subscriber rclcpp sensor_msgs nav_msgs ros2_interfaces)
# install(TARGETS yolo_subscriber DESTINATION lib/${PROJECT_NAME})

# offboard/offboard_control
include_directories(
	src/offboard_control_t
  ${Eigen3_INCLUDE_DIRS}  # Add Eigen include directories
)
file(GLOB SOURCES "src/offboard_control_t/*.cpp")
add_executable(Gps_Test_Node ${SOURCES})
ament_target_dependencies(Gps_Test_Node rclcpp mavros_msgs geometry_msgs nav_msgs geographic_msgs trajectory_msgs Eigen3 vision_msgs visualization_msgs sensor_msgs OpenCV cv_bridge ament_index_cpp ${PAL_STATISTICS_DEPS})
target_link_libraries(Gps_Test_Node yaml-cpp)  
install(TARGETS Gps_Test_Node DESTINATION lib/${PROJECT_NAME})

add_executable(gps_status src/listeners_t/GpsStatus_listeners.cpp)
ament_target_dependencies(gps_status rclcpp mavros_msgs geometry_msgs nav_msgs geographic_msgs trajectory_msgs Eigen3 vision_msgs visualization_msgs sensor_msgs OpenCV cv_bridge ament_index_cpp)
install(TARGETS gps_status DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 ardupilot_msgs geometry_msgs sensor_msgs nav_msgs)

ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(frame_transforms_t)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS frame_transforms_t
	EXPORT export_frame_transforms
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install tests
install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

############
# Testing ##
############

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	#set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	#set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

# ###########
# # Python ##
# ###########

# # Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# # Install Python executables
# install(PROGRAMS
#   src/offboard_py/offboard_control.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
