#include "pose_transformer.hpp"

PoseTransformer::PoseTransformer() : pose_msg_(std::vector<float>())

{
  pose_node_ = std::make_shared<rclcpp::Node>("pose_node");
  pose_sub_ = pose_node_->create_subscription<geometry_msgs::msg::Pose>(
      "object_pose", 10, std::bind(&PoseTransformer::pose_estimation_callback, this, std::placeholders::_1));

  // he_calib should probably be imported from a config file at some point
  he_calibration_mat_ = Eigen::Affine3f{
      Eigen::Translation3f{Eigen::Vector3f{0.18817, 0.062931, 0.6889065}} *
      (Eigen::AngleAxisf(3.0516253, Eigen::Vector3f({-0.690238, 0.707627, -0.15111}))).toRotationMatrix()};
}

void PoseTransformer::pose_estimation_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
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

std::vector<float> PoseTransformer::get_pose_msg()
{
  return pose_msg_;
}

std::vector<float> PoseTransformer::chessboard_pose_to_base_frame(float z_offset, bool Euler_angles)
{
  rclcpp::spin_some(pose_node_);

  Eigen::Quaternionf pose_quat;
  pose_quat.x() = pose_msg_[3];
  pose_quat.y() = pose_msg_[4];
  pose_quat.z() = pose_msg_[5];
  pose_quat.w() = pose_msg_[6];
  pose_quat.normalize();

  Eigen::Quaternionf transf_quat{Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())};
  Eigen::Quaternionf roatated_quat{pose_quat * transf_quat};
  // "graspable" pose in camera frame
  Eigen::Affine3f obj_in_cam{
      Eigen::Translation3f{Eigen::Vector3f{pose_msg_[0], pose_msg_[1], pose_msg_[2]}} * roatated_quat.toRotationMatrix()};

  Eigen::Affine3f obj_in_base = apply_he_calibration(obj_in_cam);

  Eigen::Vector3f obj_in_base_t{obj_in_base.translation()};

  if (Euler_angles)
  {
    auto eulerZYX = obj_in_base.rotation().eulerAngles(2, 1, 0);

    std::vector<float>
        pose_vec{obj_in_base_t[0], obj_in_base_t[1], obj_in_base_t[2], eulerZYX[0], eulerZYX[1], eulerZYX[2]};
    return pose_vec;
  }
  else
  {
    Eigen::Quaternionf obj_in_base_quat{obj_in_base.rotation()};
    std::vector<float> pose_vec{
        obj_in_base_t.x(), obj_in_base_t.y(), obj_in_base_t.z(), obj_in_base_quat.x(), obj_in_base_quat.y(), obj_in_base_quat.z(), obj_in_base_quat.w()};
    return pose_vec;
  }
}

Eigen::Affine3f PoseTransformer::apply_he_calibration(Eigen::Affine3f obj_in_cam)
{
  return he_calibration_mat_ * obj_in_cam;
}