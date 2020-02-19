#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

class PoseListener  // : public rclcpp::Node
{
public:
  PoseListener();

  std::vector<float> get_pose_msg();
  std::vector<float> get_graspable_chessboard_bose(float z_offset, bool Euler_angles);
  Eigen::Affine3f apply_he_calibration(Eigen::Affine3f);

private:
  void pose_estimation_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  std::shared_ptr<rclcpp::Node> pose_node_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Pose>> pose_sub_;
  std::vector<float> pose_msg_;
  Eigen::Affine3f he_calibration_mat_;
};
