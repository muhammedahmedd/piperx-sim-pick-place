#ifndef PIPERX_SIM_CONTROL__PIPERX_SIM_CONTROL_HPP_
#define PIPERX_SIM_CONTROL__PIPERX_SIM_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class PiperXSimControl : public rclcpp::Node
{
public:

  PiperXSimControl();

  ~PiperXSimControl() = default;

  void markerPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  void initializeMoveIt();

  void runStateMachine();

  void moveTcpToMarker();

  void moveArmJoints(const std::vector<double> & joint_angles);

  void moveGripperJoints(const std::vector<double> & joint_angles);


private:

  enum class PickState
  {
    MOVE_TO_SCAN,
    OPEN_GRIPPER,
    WAIT_FOR_MARKER,
    MOVE_TO_PICK,
    CLOSE_GRIPPER,
    LIFT,
    DONE
  };

  PickState current_state_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_sub_;

  geometry_msgs::msg::PoseStamped marker_pose_;

  bool has_marker_pose_;

  int required_marker_samples_;
  int marker_sample_count_;

  double marker_sum_x_;
  double marker_sum_y_;
  double marker_sum_z_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

  moveit::planning_interface::MoveGroupInterface::Plan arm_plan_;

  moveit::planning_interface::MoveGroupInterface::Plan gripper_plan_;

  std::vector<double> scan_pose_joints_ = {0.0, 0.373, -1.283, 1.315, 0.0, 0.0};

  std::vector<double> gripper_open_joints_ = {0.050, -0.050};

  std::vector<double> gripper_grasp_joints_;
};

#endif 