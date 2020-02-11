#pragma once

#include <xtensor/xarray.hpp>
#include <xtensor/xindex_view.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xio.hpp>
#include <xtensor-blas/xlinalg.hpp>

#include <Zivid/Zivid.h>
#include <opencv2/opencv.hpp>

namespace CPE
{
class ChessboardPoseEstimator
{
public:
	xt::xarray<float> board_pose_;
	cv::Mat rgb_;
	xt::xarray<float> xyz_;
	xt::xarray<float> corner_array_;
	xt::xarray<float> feature_pnt_cld_;


	ChessboardPoseEstimator(cv::Mat img, xt::xarray<float> xyz);
	ChessboardPoseEstimator();
	~ChessboardPoseEstimator();

	
	std::vector<float> estimate_pose();
	void find_corners();
	void extract_feature_pnt_cld();
	void show_img();
	void set_point_cloud(cv::Mat img, xt::xarray<float> xyz);
};

//utility functions

xt::xarray<float> plane_fit(xt::xarray<float> feature_pnt_cld);
xt::xarray<float> generate_xyz_xarray(Zivid::PointCloud point_cloud);
cv::Mat generate_cv_img(Zivid::PointCloud point_cloud);
std::vector<float> as_ros_pose_msg(xt::xarray<float> hom_pose_mat);

} // namespace CPE