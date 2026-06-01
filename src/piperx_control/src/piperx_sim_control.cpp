#include "piperx_sim_control/piperx_sim_control.hpp"

PiperXSimControl::PiperXSimControl() : Node("piperx_sim_control")
{
  RCLCPP_INFO(this->get_logger(), "Control node has started......");

  current_state_ = PickState::MOVE_TO_SCAN;

  has_marker_pose_ = false;

  marker_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/aruco/marker_pose_base",
    10,
    std::bind(&PiperXSimControl::markerPoseCallback, this, std::placeholders::_1)
  );
}

void PiperXSimControl::markerPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  marker_pose_ = *msg;

  has_marker_pose_ = true;

  RCLCPP_INFO(
    this->get_logger(),
    "Marker pose in %s: x=%.3f, y=%.3f, z=%.3f",
    msg->header.frame_id.c_str(),
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z
  );
}

void PiperXSimControl::initializeMoveIt()
{
  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "arm");

  gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "gripper");
}

void PiperXSimControl::runStateMachine()
{
  switch (current_state_)
  {
    case PickState::MOVE_TO_SCAN:
      RCLCPP_INFO(this->get_logger(), "State: MOVE_TO_SCAN");

      moveArmJoints(scan_pose_joints_);

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
        target_marker_pose_ = marker_pose_;
        RCLCPP_INFO(
          this->get_logger(),
          "Target marker pose frozen: x=%.3f, y=%.3f, z=%.3f",
          target_marker_pose_.pose.position.x,
          target_marker_pose_.pose.position.y,
          target_marker_pose_.pose.position.z
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

  node->runStateMachine();
  node->runStateMachine();

  rclcpp::spin_some(node);

  node->runStateMachine();

  rclcpp::shutdown();
  
  return 0;
}