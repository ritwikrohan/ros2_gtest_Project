#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"


using std::placeholders::_1;
using std::placeholders::_2;


class WaypointActionClass : public rclcpp::Node {
public:
  using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandleWaypointAction =
      rclcpp_action::ServerGoalHandle<WaypointAction>;

  WaypointActionClass(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("tortoisebot_as", options) {

    action_server = rclcpp_action::create_server<WaypointAction>(
        this, "tortoisebot_as",
        std::bind(&WaypointActionClass::handle_goal, this, _1, _2),
        std::bind(&WaypointActionClass::handle_cancel, this, _1),
        std::bind(&WaypointActionClass::handle_accepted, this, _1));
    pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&WaypointActionClass::odom_callback, this, _1));
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    curr = msg->pose.pose.position;
    tf2::Quaternion temp_yaw;
    tf2::convert(msg->pose.pose.orientation, temp_yaw);
    __yaw = tf2::getYaw(temp_yaw);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const WaypointAction::Goal> goal) {
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void
  handle_accepted(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {
    std::thread{std::bind(&WaypointActionClass::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {

    const auto d_precision = 0.05;
    const auto y_precision = M_PI / 90;

    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<WaypointAction::Feedback>();
    auto result = std::make_shared<WaypointAction::Result>();

    rclcpp::Rate loop_rate(25);

    _des_pos = goal->position;
    double desired_yaw = atan2(_des_pos.y - curr.y, _des_pos.x - curr.x);
    double desired_final_yaw = _des_pos.z;
    double err_pos =
        sqrt(pow(_des_pos.y - curr.y, 2) + pow(_des_pos.x - curr.x, 2));
    double err_yaw = desired_yaw - __yaw;
    double err_final_yaw = desired_final_yaw - __yaw;

    bool success = true;
    bool xy_goal_reached = false;

    while ((!xy_goal_reached || fabs(err_final_yaw) > y_precision) && success) {
      desired_yaw = atan2(_des_pos.y - curr.y, _des_pos.x - curr.x);
      err_yaw = desired_yaw - __yaw;
      err_pos = sqrt(pow(_des_pos.y - curr.y, 2) + pow(_des_pos.x - curr.x, 2));
      err_final_yaw = desired_final_yaw - __yaw;
      xy_goal_reached = err_pos < d_precision;

      RCLCPP_INFO(this->get_logger(), "Current Yaw: %f", __yaw);
      RCLCPP_INFO(this->get_logger(), "Desired Yaw: %f", desired_yaw);
      RCLCPP_INFO(this->get_logger(), "Error Yaw: %f", err_yaw);
      RCLCPP_INFO(this->get_logger(), "Error err_final_yaw: %f", err_final_yaw);

      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Goal canceled/preempted");
        goal_handle->canceled(result);
        success = false;
      } else if (fabs(err_yaw) > y_precision && !xy_goal_reached) {
        RCLCPP_INFO(this->get_logger(), "Fix yaw");
        _state = "fix yaw";
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = (err_yaw > 0) ? 0.35 : -0.35;
        pub->publish(twist_msg);
      } else if (!xy_goal_reached) {
        RCLCPP_INFO(this->get_logger(), "Go to point");
        _state = "go to point";
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.2;
        twist_msg.angular.z = (err_yaw > 0) ? 0.1 : -0.1;
        pub->publish(twist_msg);
      } else {
        RCLCPP_INFO(this->get_logger(), "Turning at final point");
        _state = "turning at final point";
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = (err_final_yaw > 0) ? 0.35 : -0.35;
        pub->publish(twist_msg);
      }

      feedback->position = curr;
      feedback->state = _state;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "Stop robot");
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0;
    twist_msg.angular.z = 0;
    pub->publish(twist_msg);

    if (success) {
      result->success = true;
      goal_handle->succeed(result);
    }
  }

private:
  float __yaw;
  rclcpp_action::Server<WaypointAction>::SharedPtr action_server;
  geometry_msgs::msg::Twist cmd_vel_msg;
  geometry_msgs::msg::Point curr;
//   float _yaw_precision;
  std::string _state;
  geometry_msgs::msg::Point _des_pos;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto server = std::make_shared<WaypointActionClass>();
  rclcpp::spin(server);
  rclcpp::shutdown();
  return 0;
}
