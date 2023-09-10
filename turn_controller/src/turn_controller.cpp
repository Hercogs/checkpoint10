#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

using geometry_msgs::msg::Point32;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

#define KP 1.1
#define KI 0.35
#define KD 0.2

#define II_limit 0.7
#define II_damping 0.95

#define MAX_SPEED 1.2 // [rad/sec]

class TurnController : public rclcpp::Node {
public:
  TurnController() : Node("turn_controller") {

    this->sub_odom = this->create_subscription<Odometry>(
        "/odometry/filtered", 3,
        std::bind(&TurnController::odom_clb, this, std::placeholders::_1));
    this->pub_twist = this->create_publisher<Twist>("/cmd_vel", 3);

    this->Kp = KP;
    this->Ki = KI;
    this->Kd = KD;
    this->II = 0.0;

    // Generate setpoints
    this->generate_setpoints();

    this->last_time = this->get_clock()->now();

    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "TurnController configured!");
  }

private:
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist;
  Twist cmd_msg;
  rclcpp::Time last_time;
  Odometry last_position;

  // Setpoints
  std::vector<Point32> set_points;
  std::vector<Point32>::iterator set_point;

  bool robot_stoped = false;

  // PID parameters
  float Kp, Ki, Kd;
  float II;
  float last_err;
  float err_limit = 0.05; // [rad]

  bool first_PID_iter = true;
  bool robot_failed = false;
  bool robot_at_target = false;

  void odom_clb(const Odometry::SharedPtr msg) {

    RCLCPP_INFO_ONCE(this->get_logger(), "Robot is at x: %.2f y: %.2f",
                     msg->pose.pose.position.x, msg->pose.pose.position.y);

    this->is_robot_stoped(msg);

    if (this->robot_at_target) {
      if (!this->get_next_setpoint()) {
        return;
      } else {
        this->robot_at_target = false;
      }
    }

    // Do PID loop
    if (!this->robot_failed) {
      this->PID_controller(msg);
    } else {
      RCLCPP_ERROR_ONCE(this->get_logger(), "Robot is failed");
    }

    this->last_position.pose = msg->pose;
    this->last_time = this->get_clock()->now();
  }

  void PID_controller(const Odometry::SharedPtr msg) {
    float d_x = this->set_point->x - msg->pose.pose.position.x;
    float d_y = this->set_point->y - msg->pose.pose.position.y;

    float setpoint_angle = std::atan2(d_y, d_x);

    // Get current angle
    tf2::Quaternion q;
    tf2::convert(msg->pose.pose.orientation, q);

    tf2::Matrix3x3 mat(q);

    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    float error = setpoint_angle - yaw;

    if (this->first_PID_iter) {
      this->last_err = error;
      this->first_PID_iter = false;
      this->II = 0;
      return;
    }

    if (std::abs(error) < this->err_limit) {
      if (!this->robot_stoped) {
        this->stop_robot();
        return;
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "Setpoint reached with error: %.2f radians", error);
        this->robot_at_target = true;

        return;
      }
    }

    if ((std::abs(error) - std::abs(this->last_err)) > 0.03) {
      RCLCPP_WARN(
          this->get_logger(),
          "Error getting bigger: %.2f > %.2f, possible overshoot or "
          "wrong setpoint or wrong initial position, or noisy measurment. ",
          error, this->last_err);
      //   this->robot_failed = true;
      //   this->stop_robot();
      //   return;
    }

    II = II + (error * (this->get_clock()->now().seconds() -
                        this->last_time.seconds()));
    II = (II > II_limit) ? II_limit : II;
    II *= II_damping;

    float K = Kp * error;
    float I = Ki * II;
    float D = Kd * (error - this->last_err) /
              (this->get_clock()->now().seconds() - this->last_time.seconds());

    float u = K + I + D;

    u = (std::abs(u) > MAX_SPEED) ? MAX_SPEED * std::abs(u) / u : u;

    this->cmd_msg.linear.x = 0.0;
    this->cmd_msg.angular.z = u;
    this->pub_twist->publish(this->cmd_msg);

    this->last_err = error;
    RCLCPP_INFO(this->get_logger(), "Error: %.2f deg", error * 360 / 2 / 3.14);
    RCLCPP_INFO(this->get_logger(),
                "K: : %.2f, I: : %.2f, D: %.2f, II: %.2f, u: %.2f", K, I, D, II,
                u);
  }

  void stop_robot() {
    this->cmd_msg.linear.x = 0.0;
    this->cmd_msg.angular.z = 0.0;
    this->pub_twist->publish(this->cmd_msg);
  }

  void is_robot_stoped(const Odometry::SharedPtr msg) {
  
    // Get current angle
    tf2::Quaternion q, q_last;
    tf2::convert(msg->pose.pose.orientation, q);
    tf2::convert(this->last_position.pose.pose.orientation, q_last);

    tf2::Matrix3x3 mat(q);
    tf2::Matrix3x3 mat_last(q_last);

    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    double roll_last, pitch_last, yaw_last;
    mat_last.getRPY(roll_last, pitch_last, yaw_last);

    float dis = yaw - yaw_last;
    float speed =
        dis / (this->get_clock()->now().seconds() - this->last_time.seconds());

    this->robot_stoped = (speed < 0.01) ? true : false;
  }

  void generate_setpoints() {
    RCLCPP_INFO(this->get_logger(), "Hard coding setpoint values...");

    Point32 p1, p2, p3;
    p1.x = -50.0;
    p1.y = 5.0;
    p2.x = 50.0;
    p2.y = 5.0;
    p3.x = 0.0;
    p3.y = 50.0;

    this->set_points.push_back(p1);
    this->set_points.push_back(p2);
    this->set_points.push_back(p3);

    int p_idx = 1;
    for (auto i : this->set_points) {

      RCLCPP_INFO(this->get_logger(), "Number %d setpoint: %.1f, %.1f", p_idx++,
                  i.x, i.y);
    }

    this->set_point = this->set_points.begin();
    this->first_PID_iter = true;

    RCLCPP_INFO(this->get_logger(), "The first setpoint is : %.1f, %.1f",
                this->set_point->x, this->set_point->y);
  }

  bool get_next_setpoint() {
    if (this->set_point == this->set_points.end() - 1) {
      RCLCPP_INFO_ONCE(this->get_logger(), "All setpoints reached sucessfully");
      return false;
    }

    this->set_point++;

    rclcpp::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(this->get_logger(), "The next setpoint is : %.1f, %.1f",
                this->set_point->x, this->set_point->y);

    first_PID_iter = true;
    return true;
  }
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  std::shared_ptr<TurnController> node = std::make_shared<TurnController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}