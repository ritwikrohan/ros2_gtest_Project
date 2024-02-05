cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
  DEPENDENCIES geometry_msgs
)
include_directories(include)


add_executable(tortoisebot_action_server src/tortoisebot_action_server.cpp)
ament_target_dependencies(tortoisebot_action_server
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
)
rosidl_target_interfaces(tortoisebot_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS
	tortoisebot_action_server
	DESTINATION lib/${PROJECT_NAME}
)

add_library(tortoisebot_action_server_gtest src/tortoisebot_action_server.cpp)
ament_target_dependencies(tortoisebot_action_server_gtest 
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
)

rosidl_target_interfaces(tortoisebot_action_server_gtest
      ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
  	  test/main.cpp
      test/waypoints_ros2_test.cpp
  )
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_link_libraries(${PROJECT_NAME}_test tortoisebot_action_server_gtest)
  ament_target_dependencies(${PROJECT_NAME}_test 
	rclcpp
	rclcpp_action
	std_msgs
	geometry_msgs
	nav_msgs
	tf2
	std_srvs
  )
  install(TARGETS
        ${PROJECT_NAME}_test
        DESTINATION lib/${PROJECT_NAME})
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
endif()

ament_package()
