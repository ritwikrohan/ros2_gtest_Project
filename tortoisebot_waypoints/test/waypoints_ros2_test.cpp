#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"
#include "gtest/gtest.h"
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
using std_srvs::srv::Empty;

class TestSetup {
public:
  TestSetup() { rclcpp::init(0, nullptr); }
};

class WaypointTesting : public ::testing::Test {
public:
  using WaypointTestAction = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandleTest = rclcpp_action::ClientGoalHandle<WaypointTestAction>;

  WaypointTesting() {

    test_node = rclcpp::Node::make_shared("test_node");
    odom_subscription = test_node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&WaypointTesting::odom_data_callback, this, _1));

    action_test_client = rclcpp_action::create_client<WaypointTestAction>(
        test_node, "tortoisebot_as");

    srv_client_ = test_node->create_client<Empty>("/reset_world");

    this->goal_point.x = 1.0;
    this->goal_point.y = 1.0;
    this->goal_point.z = 1.57;

    if (!action_test_client->wait_for_action_server(10s)) {
      RCLCPP_ERROR(test_node->get_logger(), "Action server not available");
    }

    auto goal_msg = WaypointTestAction::Goal();
    // reset_world(); 

    goal_msg.position = this->goal_point;
    auto goal_options =
        rclcpp_action::Client<WaypointTestAction>::SendGoalOptions();
    goal_options.goal_response_callback =
        std::bind(&WaypointTesting::goal_response_handler, this, _1);
    goal_options.feedback_callback =
        std::bind(&WaypointTesting::feedback_handler, this, _1, _2);
    goal_options.result_callback =
        std::bind(&WaypointTesting::result_handler, this, _1);
    auto future = action_test_client->async_send_goal(goal_msg, goal_options);
  }

  bool CheckPositionValidity() {
    while (is_test_active) {
      rclcpp::spin_some(test_node);
    }

    float x_error = abs(goal_point.x - current_point.x);
    float y_error = abs(goal_point.y - current_point.y);
    float linear_error = std::sqrt(std::pow(x_error, 2) + std::pow(y_error, 2));
    std::cout << linear_error << std::endl << std::flush;
    if (linear_error < 0.2) {
      return true;
    }

    return false;
  }

  bool CheckAngleValidity() {
    while (is_test_active) {
      rclcpp::spin_some(test_node);
    }

    float yaw_error = abs(goal_point.z - current_yaw);
    std::cout << yaw_error << std::endl << std::flush;
    if (yaw_error <= 0.5) {
      return true;
    }

    return false;
  }

private:
  std::shared_ptr<rclcpp::Node> test_node;

  void
  goal_response_handler(std::shared_future<GoalHandleTest::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(test_node->get_logger(), "Goal rejected!");
    } else {
      RCLCPP_INFO(test_node->get_logger(), "Goal accepted!");
    }
  }

  void feedback_handler(
      GoalHandleTest::SharedPtr,
      const std::shared_ptr<const WaypointTestAction::Feedback> feedback) {}

  void result_handler(const GoalHandleTest::WrappedResult &result) {
    is_test_active = false;
  }

  void odom_data_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_point = msg->pose.pose.position;
    tf2::Quaternion current_angle;
    tf2::convert(msg->pose.pose.orientation, current_angle);
    current_yaw = tf2::getYaw(current_angle);
  }

  void reset_world() {
    std::cout << "RESet..." << std::endl;
    auto request = std::make_shared<Empty::Request>();
    auto result_future = srv_client_->async_send_request(request);
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    auto timeout =
        std::chrono::seconds(10); 
    auto result =
        rclcpp::spin_until_future_complete(test_node, result_future, timeout);

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(test_node->get_logger(),
                   "Failed to call /reset_world service within the timeout");
    }
    // else {
    // Reset current_point and current_yaw to zero
    //   current_point.x = 0.0;
    //   current_point.y = 0.0;
    //   current_point.z = 0.0;
    //   current_yaw = 0.0;
    // }
  }

  float goal_yaw = 0.0;
  bool is_test_active = true;
  bool is_setup_performed = false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
  geometry_msgs::msg::Point goal_point;
  geometry_msgs::msg::Point current_point;
  float current_yaw;
  rclcpp_action::Client<WaypointTestAction>::SharedPtr action_test_client;
  rclcpp::TimerBase::SharedPtr test_timer;
  rclcpp::Client<Empty>::SharedPtr srv_client_;
};

TestSetup global_test_setup;

TEST_F(WaypointTesting, CheckPositionValidity) {
  EXPECT_TRUE(CheckPositionValidity());
}

TEST_F(WaypointTesting, CheckAngleValidity) {
  EXPECT_TRUE(CheckAngleValidity());
}
