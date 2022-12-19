#include <cstdio>
#include <functional>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "create_msgs/msg/bumper.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

class BumperSubscriber : public rclcpp::Node {
 public:
  BumperSubscriber() : Node("bumper_subscriber") {
    bumper_subscription_ = this->create_subscription<create_msgs::msg::Bumper>(
        "bumper", 10, std::bind(&BumperSubscriber::bumper_callback, this, _1));
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&BumperSubscriber::odom_callback, this, _1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    is_moving_ = false;
    was_pressed_ = false;
    count_ = 0;
  }

 private:
  void bumper_callback(const create_msgs::msg::Bumper& msg) {
    bool pressed = msg.is_left_pressed || msg.is_right_pressed;
    if (pressed && !was_pressed_) {
      is_moving_ = !is_moving_;
      RCLCPP_INFO(this->get_logger(), "Bumper pressed, is_moving: %d", is_moving_);
    }
    was_pressed_ = pressed;
  }

  void odom_callback(const nav_msgs::msg::Odometry msg) {
    // Calculate robot orientation in radians
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(msg.pose.pose.orientation, tf2_quat);
    tf2::Matrix3x3 tf2_mat(tf2_quat);
    double roll, pitch, yaw;
    tf2_mat.getRPY(roll, pitch, yaw);

    ++count_;
    if (count_ > 100) {
      count_ = 0;
      // Log location
      RCLCPP_INFO(this->get_logger(), "Robot location: (%f, %f, %f)",
                  msg.pose.pose.position.x, msg.pose.pose.position.y, yaw);
    }

    if (is_moving_) {
      double distance =
          sqrt(pow(msg.pose.pose.position.x, 2) + pow(msg.pose.pose.position.y, 2));

      // Check if we're within 0.01m of the origin
      if (distance < 0.01) {
        is_moving_ = false;
        auto message = geometry_msgs::msg::Twist();
        publisher_->publish(message);
      } else {
        // Calculate angle from robot to origin
        double angle = atan2(-msg.pose.pose.position.y, -msg.pose.pose.position.x);

        // Calculate difference between current angle and angle to origin
        double angle_diff = angle - yaw;

        // Normalize angle difference to be between -pi and pi
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        float turn_speed = 3 * angle_diff;
        if (turn_speed > 0) {
          if (turn_speed < 0.2) turn_speed = 0.2;
          if (turn_speed > 4.5) turn_speed = 4.5;
        } else {
          if (turn_speed > -0.2) turn_speed = -0.2;
          if (turn_speed < -4.5) turn_speed = -4.5;
        }

        auto message = geometry_msgs::msg::Twist();
        message.angular.z = turn_speed;

        // If angle is small enough, move forward
        if (fabs(angle_diff) < std::max(0.79, distance * 0.1))
          message.linear.x =
              std::min(distance * 2.5, 0.25) / (std::min(1.0f, fabs(turn_speed) * 3));

        publisher_->publish(message);
      }
    }
  }

  rclcpp::Subscription<create_msgs::msg::Bumper>::SharedPtr bumper_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  bool is_moving_;
  bool was_pressed_;
  uint8_t count_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BumperSubscriber>());
  rclcpp::shutdown();
  return 0;
}
