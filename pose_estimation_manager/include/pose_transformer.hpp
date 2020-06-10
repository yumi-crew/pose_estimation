// Copyright 2020 Markus Bjønnes and Marius Nilsen.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

class PoseTransformer  // : public rclcpp::Node
{
public:
  PoseTransformer();

  /**
   * @returns the contents of the /object_pose topic.
  **/
  std::vector<float> get_pose_msg();

  /**
   * @returns A "graspable chessboard pose"
  **/
  std::vector<float> chessboard_pose_to_base_frame(float z_offset, bool Euler_angles);

  /**
   * Applies the current hand-eye calibration, transforming the object pose from the camera frame to the robot base frame.
  **/
  Eigen::Affine3f apply_he_calibration(Eigen::Affine3f);

  /**
   * @returns an object pose which can be used for grasping by the ABB YuMi.
  **/
  std::vector<double> obj_in_base_frame();

  /**
   * @returns grasp_pose + z_offset
  **/
  std::vector<double> hover_pose();

private:
  void pose_estimation_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  std::shared_ptr<rclcpp::Node> pose_node_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> pose_sub_;
  std::vector<float> pose_msg_;
  Eigen::Affine3f he_calibration_mat_;
};
