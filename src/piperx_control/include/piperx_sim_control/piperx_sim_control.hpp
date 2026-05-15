#ifndef PIPERX_SIM_CONTROL__PIPERX_SIM_CONTROL_HPP_
#define PIPERX_SIM_CONTROL__PIPERX_SIM_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class PiperXSimControl : public rclcpp::Node
{
public:
  PiperXSimControl();

private:
  void publish_joint_command();
  void publish_scan_pose_once();

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif 