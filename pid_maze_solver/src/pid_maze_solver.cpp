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

// Distance controller parameters
#define Kp1 0.80
#define Ki1 0.20
#define Kd1 0.15

#define II1_limit 0.6
#define II1_damping 0.95

#define max_linear_speed 0.6 // [m/sec]
// END: Distance controller parameters

// Turn controller parameters
#define Kp2 1.0
#define Ki2 0.30
#define Kd2 0.2

#define II2_limit 0.7
#define II2_damping 0.95

#define max_angular_speed 1.1 // [rad/sec]
// END: Turn controller parameters

class PIDController {
public:
  PIDController() {}
  PIDController(float Kp, float Ki, float Kd, float II_limit, float II_damping,
                float max_cmd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->II_limit = II_limit;
    this->II_damping = II_damping;
    this->max_cmd = max_cmd;

    this->II = 0;
    this->pre_err = 0;
  }

  ~PIDController() {}

  void reset() {
    this->II = 0;
    this->pre_err = 0;
  }

  float update(float err, double dt) {

    this->II += (err * dt);
    this->II = (this->II > this->II_limit) ? this->II_limit : this->II;
    this->II *= this->II_damping;

    float K, I, D = 0;

    K = this->Kp * err;
    I = this->Ki * this->II;

    if (dt != 0)
      D = Kd * (err - this->pre_err) / dt;

    float u = K + I + D;

    u = (std::abs(u) > this->max_cmd) ? this->max_cmd * std::abs(u) / u : u;

    this->pre_err = err;

    return u;
  }

private:
  float Kp, Ki, Kd;
  float II;
  float II_limit, II_damping;
  float max_cmd;
  float pre_err;
};

class PidMazeSolver : public rclcpp::Node {
public:
  PidMazeSolver() : Node("pid_maze_solver") {

    this->sub_odom = this->create_subscription<Odometry>(
        "/odometry/filtered", 3,
        std::bind(&PidMazeSolver::odom_clb, this, std::placeholders::_1));
    this->pub_twist = this->create_publisher<Twist>("/cmd_vel", 3);

    this->pid_distance =
        PIDController(Kp1, Ki1, Kd1, II1_limit, II1_damping, max_linear_speed);
    this->pid_turn =
        PIDController(Kp2, Ki2, Kd2, II2_limit, II2_damping, max_angular_speed);

    // Generate setpoints
    this->generate_setpoints();

    this->last_time = this->get_clock()->now();

    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "PidMazeSolver configured!");
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
  bool setpoint_type = false; // true for angular, false for linear

  PIDController pid_distance, pid_turn;

  bool robot_stoped = true;

  float last_err_linear;
  float err_limit_rad = 0.05;    // [rad]
  float err_limit_meters = 0.04; // [rad]

  bool first_PID_iter = true;
  bool robot_failed = false;
  bool robot_at_target = false;

  void odom_clb(const Odometry::SharedPtr msg) {

    RCLCPP_INFO_ONCE(this->get_logger(), "Robot is at x: %.2f y: %.2f",
                     msg->pose.pose.position.x, msg->pose.pose.position.y);

    if (this->robot_at_target) {
      if (!this->get_next_setpoint()) {
        return;
      } else {
        this->robot_at_target = false;
        this->pid_distance.reset();
        this->pid_turn.reset();
      }
    }

    // Calculate time difference
    double dt = this->get_clock()->now().seconds() - this->last_time.seconds();

    // Update robot moving status
    this->is_robot_stoped(msg, dt);

    float linear_rate = 0.0, angular_rate = 0.0;

    // Get current angle
    float current_yaw = this->get_yaw(msg);
    // Get setpoint angle
    float d_x = this->set_point->x - msg->pose.pose.position.x;
    float d_y = this->set_point->y - msg->pose.pose.position.y;
    float setpoint_yaw = std::atan2(d_y, d_x);

    float error_angle = setpoint_yaw - current_yaw;

    if (this->setpoint_type) {
      // Angular setpoint - turn robot
      if (std::abs(error_angle) < this->err_limit_rad) {
        if (!this->robot_stoped) {
          this->stop_robot();
        } else {
          RCLCPP_INFO(this->get_logger(),
                      "Setpoint reached with error_angle: %.2f radians",
                      error_angle);
          this->robot_at_target = true;
          this->stop_robot();
        }
      } else {
        angular_rate = this->pid_turn.update(error_angle, dt);
      }
    } else {
      // Linear setpoint - move and turn robot
      float error_linear = std::sqrt(std::pow(d_x, 2) + std::pow(d_y, 2));
      RCLCPP_INFO(this->get_logger(), " error_linear: %.2f", error_linear);

      if ((std::abs(error_linear) < this->err_limit_meters) ||
          ((std::abs(error_linear) < (4 * this->err_limit_meters)) &&
           error_linear > this->last_err_linear)) {
        if (!this->robot_stoped) {
          this->stop_robot();
        } else {
          RCLCPP_INFO(this->get_logger(),
                      "Setpoint reached with error_linear: %.2f", error_linear);
          this->robot_at_target = true;
          this->stop_robot();
          // return;
        }
        this->last_err_linear = error_linear;
      } else {
        // To avoid sharp turns next to setpoint, then regulate angl;e only in
        // distance
        if (std::abs(error_linear) > 0.2) {
          angular_rate = this->pid_turn.update(error_angle, dt);
        }
        linear_rate = this->pid_distance.update(error_linear, dt);
      }
    }

    // Publish twist
    this->cmd_msg.linear.x = linear_rate;
    this->cmd_msg.angular.z = angular_rate;
    this->pub_twist->publish(this->cmd_msg);

    // Save last measurments and values
    this->last_position.pose = msg->pose;
    this->last_time = this->get_clock()->now();
  }

  void stop_robot() {
    this->cmd_msg.linear.x = 0.0;
    this->cmd_msg.angular.z = 0.0;
    this->pub_twist->publish(this->cmd_msg);
  }

  void is_robot_stoped(const Odometry::SharedPtr msg, double dt) {

    // Get current angle
    float current_yaw = this->get_yaw(msg);
    float pre_yaw = this->get_yaw(this->last_position);

    float dis_angle = current_yaw - pre_yaw;
    float speed_angle = dis_angle / dt;

    float d_x =
        msg->pose.pose.position.x - this->last_position.pose.pose.position.x;
    float d_y =
        msg->pose.pose.position.y - this->last_position.pose.pose.position.y;

    float dis_linear = std::sqrt(std::pow(d_x, 2) + std::pow(d_y, 2));
    float speed_linear = dis_linear / dt;

    if (speed_linear < 0.02 && speed_angle < 0.02) {
      this->robot_stoped = true;
    } else {
      this->robot_stoped = false;
    }
  }

  float get_yaw(const Odometry::SharedPtr msg) {
    tf2::Quaternion q;
    tf2::convert(msg->pose.pose.orientation, q);

    tf2::Matrix3x3 mat(q);

    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    yaw = std::fmod(yaw, 360);
    yaw = std::fmod((yaw + 360), 360);
    if (yaw > 180)
      yaw -= 360;

    return yaw;
  }

  float get_yaw(const Odometry msg) {
    tf2::Quaternion q;
    tf2::convert(msg.pose.pose.orientation, q);

    tf2::Matrix3x3 mat(q);

    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    yaw = std::fmod(yaw, 360);
    yaw = std::fmod((yaw + 360), 360);
    if (yaw > 180)
      yaw -= 360;

    return yaw;
  }

  void generate_setpoints() {
    RCLCPP_INFO(this->get_logger(), "Hard coding setpoint values...");

    Point32 p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15,
        p16, p17, p18, p19, p20, p21, p22, p23, p24, p25, p26, p27, p28;
    p1.x = 0.35;
    p1.y = 0;
    p1.z = -1;

    p2.x = p1.x;
    p2.y = p1.y;
    p2.z = 0;

    p3.x = 0.53;
    p3.y = -0.15;
    p3.z = -1;

    p4.x = p3.x;
    p4.y = p3.y;
    p4.z = 0;

    p5.x = 0.53;
    p5.y = -1.35;
    p5.z = -1;

    p6.x = p5.x;
    p6.y = p5.y;
    p6.z = 0;

    p7.x = 1.05;
    p7.y = -1.35;
    p7.z = -1;

    p8.x = p7.x;
    p8.y = p7.y;
    p8.z = 0;

    p9.x = 1.05;
    p9.y = -0.80;
    p9.z = -1;

    p10.x = p9.x;
    p10.y = p9.y;
    p10.z = 0;

    p11.x = 1.43;
    p11.y = -0.80;
    p11.z = -1;

    p12.x = p11.x;
    p12.y = p11.y;
    p12.z = 0;

    p13.x = 1.45;
    p13.y = -0.30;
    p13.z = -1;

    p14.x = p13.x;
    p14.y = p13.y;
    p14.z = 0;

    p15.x = 1.98;
    p15.y = -0.3;
    p15.z = -1;

    p16.x = p15.x;
    p16.y = p15.y;
    p16.z = 0;

    p17.x = 1.98;
    p17.y = 0.50;
    p17.z = -1;

    p18.x = p17.x;
    p18.y = p17.y;
    p18.z = 0;

    p19.x = 1.66;
    p19.y = 0.50;
    p19.z = -1;

    p20.x = p19.x;
    p20.y = p19.y;
    p20.z = 0;

    p21.x = 1.66;
    p21.y = 0.2;
    p21.z = -1;

    p22.x = p21.x;
    p22.y = p21.y;
    p22.z = 0;

    p23.x = 1.05;
    p23.y = 0.2;
    p23.z = -1;

    p24.x = p23.x;
    p24.y = p23.y;
    p24.z = 0;

    p25.x = 0.7;
    p25.y = 0.5;
    p25.z = -1;

    p26.x = p25.x;
    p26.y = p25.y;
    p26.z = 0;

    p27.x = 0.2;
    p27.y = 0.5;
    p27.z = -1;

    p28.x = p27.x;
    p28.y = p27.y;
    p28.z = 0;

    this->set_points.push_back(p1);
    this->set_points.push_back(p2);
    this->set_points.push_back(p3);
    this->set_points.push_back(p4);
    this->set_points.push_back(p5);
    this->set_points.push_back(p6);
    this->set_points.push_back(p7);
    this->set_points.push_back(p8);
    this->set_points.push_back(p9);
    this->set_points.push_back(p10);
    this->set_points.push_back(p11);
    this->set_points.push_back(p12);
    this->set_points.push_back(p13);
    this->set_points.push_back(p14);
    this->set_points.push_back(p15);
    this->set_points.push_back(p16);
    this->set_points.push_back(p17);
    this->set_points.push_back(p18);
    this->set_points.push_back(p19);
    this->set_points.push_back(p20);
    this->set_points.push_back(p21);
    this->set_points.push_back(p22);
    this->set_points.push_back(p23);
    this->set_points.push_back(p24);
    this->set_points.push_back(p25);
    this->set_points.push_back(p26);
    this->set_points.push_back(p27);
    this->set_points.push_back(p28);

    int p_idx = 1;
    for (auto i : this->set_points) {

      RCLCPP_INFO(this->get_logger(), "Number %d setpoint: %.1f, %.1f", p_idx++,
                  i.x, i.y);
    }

    this->set_point = this->set_points.begin();
    this->first_PID_iter = true;

    if (this->set_point->z < 0) {
      this->setpoint_type = true;
    } else {
      this->setpoint_type = false;
    }

    const char *type_string;
    type_string = (this->setpoint_type) ? "turning " : "linear";

    RCLCPP_INFO(this->get_logger(), "The first setpoint (%s) is : %.1f, %.1f",
                type_string, this->set_point->x, this->set_point->y);
  }

  bool get_next_setpoint() {
    if (this->set_point == this->set_points.end() - 1) {
      RCLCPP_INFO_ONCE(this->get_logger(), "All setpoints reached sucessfully");
      return false;
    }

    this->set_point++;

    if (this->set_point->z < 0) {
      this->setpoint_type = true;
    } else {
      this->setpoint_type = false;
    }

    const char *type_string;
    type_string = (this->setpoint_type) ? "turning " : "linear";

    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(this->get_logger(), "The next setpoint (%s) is : %.1f, %.1f",
                type_string, this->set_point->x, this->set_point->y);

    first_PID_iter = true;
    return true;
  }
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  std::shared_ptr<PidMazeSolver> node = std::make_shared<PidMazeSolver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}