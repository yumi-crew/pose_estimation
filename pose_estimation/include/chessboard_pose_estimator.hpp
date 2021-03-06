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
#include <iostream>
#include <math.h>
#include <vector>

#include <xtensor/xarray.hpp>
#include <xtensor/xindex_view.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xio.hpp>
#include <xtensor-blas/xlinalg.hpp>

#include <Zivid/Zivid.h>
#include <opencv2/opencv.hpp>

namespace pose_estimation
{
class ChessboardPoseEstimator
{
private:
	xt::xarray<float> board_pose_;
	cv::Mat rgb_;
	xt::xarray<float> xyz_;
	xt::xarray<float> corner_array_;
	xt::xarray<float> feature_pnt_cld_;
	bool found_corners_;

public:
	ChessboardPoseEstimator(cv::Mat img, xt::xarray<float> xyz);
	ChessboardPoseEstimator();
	~ChessboardPoseEstimator();

	
	std::vector<float> estimate_pose();
	bool find_corners(int nx, int ny);
	void extract_feature_pnt_cld();
	void show_img();
	void set_point_cloud(xt::xarray<float> &xyz, xt::xarray<int> &rgb);
};

//utility functions

xt::xarray<float> plane_fit(xt::xarray<float> feature_pnt_cld);
xt::xarray<float> generate_xyz_xarray(Zivid::PointCloud &point_cloud);
xt::xarray<int> generate_rgb_xarray(Zivid::PointCloud &point_cloud);
cv::Mat generate_cv_img(Zivid::PointCloud &point_cloud);
cv::Mat convert_xarray_to_cv_mat(xt::xarray<int> &rgb_xarray);
std::vector<float> as_ros_pose_msg(xt::xarray<float> hom_pose_mat);

} // namespace