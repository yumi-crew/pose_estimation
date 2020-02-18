#include "pose_listener.hpp"


PoseListener::PoseListener(const std::string node_name) : rclcpp::Node(node_name),
                                                          pose_msg_(std::vector<float>())
{
  pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "object_pose", 10, std::bind(&PoseListener::pose_estimation_callback, this, std::placeholders::_1)
    );
}

void PoseListener::pose_estimation_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  pose_msg_.clear();
  pose_msg_.reserve(7);

  pose_msg_.push_back(msg->position.x);
  pose_msg_.push_back(msg->position.y);
  pose_msg_.push_back(msg->position.z);
  pose_msg_.push_back(msg->orientation.x);
  pose_msg_.push_back(msg->orientation.y);
  pose_msg_.push_back(msg->orientation.z);
  pose_msg_.push_back(msg->orientation.w);
}

std::vector<float> PoseListener::get_pose()
{
  return pose_msg_;
}