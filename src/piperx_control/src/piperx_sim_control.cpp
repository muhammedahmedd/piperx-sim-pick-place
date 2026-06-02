#include "piperx_sim_control/piperx_sim_control.hpp"


PiperXSimControl::PiperXSimControl() : Node("piperx_sim_control")
{
  RCLCPP_INFO(this->get_logger(), "Control node has started......");

  marker_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/aruco/marker_pose_base",
    1,
    std::bind(&PiperXSimControl::markerPoseCallback, this, std::placeholders::_1)
  );

  current_state_ = PickState::MOVE_TO_SCAN;

  has_marker_pose_ = false;

  required_marker_samples_ = 20;
  marker_sample_count_ = 0;

  marker_sum_x_ = 0.0;
  marker_sum_y_ = 0.0;
  marker_sum_z_ = 0.0;

}

void PiperXSimControl::markerPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (current_state_ != PickState::WAIT_FOR_MARKER)
  {
    return;
  }

  if (has_marker_pose_)
  {
    return;
  }

  marker_sum_x_ += msg->pose.position.x;
  marker_sum_y_ += msg->pose.position.y;
  marker_sum_z_ += msg->pose.position.z;

  marker_sample_count_++;

  RCLCPP_INFO(
    this->get_logger(),
    "Collected marker sample %d/%d: x=%.3f, y=%.3f, z=%.3f",
    marker_sample_count_,
    required_marker_samples_,
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z
  );

  if (marker_sample_count_ == required_marker_samples_)
  {
    marker_pose_ = *msg;

    marker_pose_.pose.position.x = marker_sum_x_ / marker_sample_count_;
    marker_pose_.pose.position.y = marker_sum_y_ / marker_sample_count_;
    marker_pose_.pose.position.z = marker_sum_z_ / marker_sample_count_;

    has_marker_pose_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Marker average ready: x=%.3f, y=%.3f, z=%.3f",
      marker_pose_.pose.position.x,
      marker_pose_.pose.position.y,
      marker_pose_.pose.position.z
    );
  }
}

void PiperXSimControl::initializeMoveIt()
{
  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "arm");

  gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "gripper");

  bool tcp_set = arm_group_->setEndEffectorLink("gripper_tcp");

  if (tcp_set)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Arm end-effector link changed to: %s",
      arm_group_->getEndEffectorLink().c_str()
    );
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to set gripper_tcp as end-effector link.");
  }
}

void PiperXSimControl::runStateMachine()
{
  switch (current_state_)
  {
    case PickState::MOVE_TO_SCAN:
      RCLCPP_INFO(this->get_logger(), "State: MOVE_TO_SCAN");

      moveArmJoints(scan_pose_joints_);

      RCLCPP_INFO(this->get_logger(), "Waiting for arm/camera to settle...");
      rclcpp::sleep_for(std::chrono::seconds(3));

      current_state_ = PickState::OPEN_GRIPPER;

      break;

    case PickState::OPEN_GRIPPER:
      RCLCPP_INFO(this->get_logger(), "State: OPEN_GRIPPER");

      moveGripperJoints(gripper_open_joints_);

      current_state_ = PickState::WAIT_FOR_MARKER;

      break; 
    
    case PickState::WAIT_FOR_MARKER:
      RCLCPP_INFO(this->get_logger(), "State: WAIT_FOR_MARKER");

      if (has_marker_pose_)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Target marker pose frozen: x=%.3f, y=%.3f, z=%.3f",
          marker_pose_.pose.position.x,
          marker_pose_.pose.position.y,
          marker_pose_.pose.position.z
        );

        current_state_ = PickState::MOVE_TO_PICK;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "No marker pose yet. Staying in WAIT_FOR_MARKER.");
      }

      break;

    default:
      RCLCPP_INFO(this->get_logger(), "State not implemented yet.");

      break;
  }
}

void PiperXSimControl::moveArmJoints(const std::vector<double> & joint_angles)
{
  arm_group_->setJointValueTarget(joint_angles);

  if (arm_group_->plan(arm_plan_) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    arm_group_->execute(arm_plan_);
  }
  else 
  {
    RCLCPP_INFO(this->get_logger(), "Arm plan failed");
  }
}

void PiperXSimControl::moveGripperJoints(const std::vector<double> & joint_angles)
{
  gripper_group_->setJointValueTarget(joint_angles);

  if (gripper_group_->plan(gripper_plan_) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    gripper_group_->execute(gripper_plan_);
  }
  else 
  {
    RCLCPP_INFO(this->get_logger(), "Gripper plan failed");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PiperXSimControl>();

  node->initializeMoveIt();

  node->runStateMachine();  // MOVE_TO_SCAN
  node->runStateMachine();  // OPEN_GRIPPER, then state becomes WAIT_FOR_MARKER

  rclcpp::Rate rate(30);

  for (int i = 0; i < 200; i++)
  {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  node->runStateMachine();  // WAIT_FOR_MARKER: freeze averaged marker pose if ready

  rclcpp::shutdown();
  
  return 0;
}