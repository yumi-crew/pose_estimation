#pragma once
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

class PoseTransformer  // : public rclcpp::Node
{
public:
  PoseTransformer();

  std::vector<float> get_pose_msg();
  std::vector<float> chessboard_pose_to_base_frame(float z_offset, bool Euler_angles);
  Eigen::Affine3f apply_he_calibration(Eigen::Affine3f);

private:
  void pose_estimation_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  std::shared_ptr<rclcpp::Node> pose_node_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Pose>> pose_sub_;
  std::vector<float> pose_msg_;
  Eigen::Affine3f he_calibration_mat_;
};
