#include "piperx_sim_control/piperx_sim_control.hpp"

PiperXSimControl::PiperXSimControl() : Node("piperx_sim_control")
{
  publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_command", 10);

  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PiperXSimControl::publish_scan_pose_once, this));

  RCLCPP_INFO(this->get_logger(), "Node started. Will publish scan pose once after 1 second.");}

void PiperXSimControl::publish_joint_command()
{
  sensor_msgs::msg::JointState msg;

  msg.header.stamp = this->now();

  msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_joint1", "gripper_joint2"};

  msg.position = {0.0, 0.373, -1.283, 1.315, 0.0, 0.0, 0.0, 0.0};

  publisher_->publish(msg);
}

void PiperXSimControl::publish_scan_pose_once()
{
  this->publish_joint_command();

  RCLCPP_INFO(this->get_logger(), "Published scan pose once.");

  timer_->cancel();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PiperXSimControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}