#include <iostream>
#include <math.h>
#include "chessboard_pose_estimator.hpp"

namespace CPE
{
ChessboardPoseEstimator::ChessboardPoseEstimator(cv::Mat img, xt::xarray<float> xyz)
	: rgb_(img), xyz_(xyz)
{
}

ChessboardPoseEstimator::ChessboardPoseEstimator()
{
}

ChessboardPoseEstimator::~ChessboardPoseEstimator()
{
}

void ChessboardPoseEstimator::set_point_cloud(cv::Mat img, xt::xarray<float> xyz)
{
	xyz_ = xyz;
	rgb_ = img;
}

void ChessboardPoseEstimator::find_corners()
{
	cv::Size patternsize(20, 13);
	std::vector<cv::Point2f> corners;
	bool found = cv::findChessboardCorners(rgb_, patternsize, corners);
	cv::drawChessboardCorners(rgb_, patternsize, cv::Mat(corners), found);

	corner_array_ = xt::zeros<float>(std::vector<size_t>{corners.size(), 2});
	for (size_t i = 0; i < corners.size(); i++)
	{
		corner_array_(i, 0) = corners[i].x;
		corner_array_(i, 1) = corners[i].y;
	}
}

void ChessboardPoseEstimator::extract_feature_pnt_cld()
{
	feature_pnt_cld_ = xt::zeros<float>(std::vector<size_t>{corner_array_.shape()[0], 3});
	float coord_val{0.0};
	int pix_x{0};
	int pix_y{0};

	for (size_t i = 0; i < corner_array_.shape()[0]; i++)
	{
		pix_x = static_cast<int>(corner_array_(i, 0));
		pix_y = static_cast<int>(corner_array_(i, 1));
		for (int j = 0; j < 3; j++)
		{
			coord_val = xyz_(pix_y, pix_x, j);
			feature_pnt_cld_(i, j) = coord_val;
			if (std::isnan(coord_val))
			{
				feature_pnt_cld_(i, j) = 0.0;
			}
			else
			{
				feature_pnt_cld_(i, j) = coord_val;
			}
		}
	}
}

std::vector<float> ChessboardPoseEstimator::estimate_pose()
{
	board_pose_ = CPE::plane_fit(feature_pnt_cld_);
	return as_ros_pose_msg(board_pose_);
}

void ChessboardPoseEstimator::show_img()
{
	cv::namedWindow("board");
	cv::imshow("board", rgb_);
	cv::waitKey(0);
}

xt::xarray<float> plane_fit(xt::xarray<float> feature_pnt_cld)
{
	auto centroid = xt::mean(feature_pnt_cld, 0);
	auto svd_res = xt::linalg::svd(feature_pnt_cld - centroid);

	auto V = std::get<2>(svd_res);

	xt::xarray<float> x_vec = xt::view(V, 0, xt::all());
	xt::xarray<float> y_vec = xt::view(V, 1, xt::all());
	xt::xarray<float> z_vec = xt::view(V, 2, xt::all());

	//create homogeneous transformation matrix to represent pose
	xt::xarray<float> pose{xt::zeros<float>({4, 4})};
	xt::view(pose, xt::range(0, 3), 0) = x_vec;
	xt::view(pose, xt::range(0, 3), 1) = y_vec;
	xt::view(pose, xt::range(0, 3), 2) = z_vec;
	xt::view(pose, xt::range(0, 3), 3) = centroid;
	pose(3, 3) = 1.0;
	std::cout << pose << std::endl;
	return pose;
}

cv::Mat generate_cv_img(Zivid::PointCloud point_cloud)
{
	xt::xarray<int> rgb{std::vector<size_t>{point_cloud.height(), point_cloud.width(), 3}};
	for (size_t i = 0; i < point_cloud.height(); i++)
	{
		for (size_t j = 0; j < point_cloud.width(); j++)
		{
			rgb(i, j, 0) = point_cloud(i, j).red();
			rgb(i, j, 1) = point_cloud(i, j).green();
			rgb(i, j, 2) = point_cloud(i, j).blue();
		}
	}
	cv::Mat img{rgb.shape()[0], rgb.shape()[1], CV_8UC3};
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			img.at<cv::Vec3b>(i, j)[0] = rgb(i, j, 0);
			img.at<cv::Vec3b>(i, j)[1] = rgb(i, j, 1);
			img.at<cv::Vec3b>(i, j)[2] = rgb(i, j, 2);
		}
	}
	return img;
}

xt::xarray<float> generate_xyz_xarray(Zivid::PointCloud point_cloud)
{
	xt::xarray<float> xyz{std::vector<size_t>{point_cloud.height(), point_cloud.width(), 3}};
	for (size_t i = 0; i < point_cloud.height(); i++)
	{
		for (size_t j = 0; j < point_cloud.width(); j++)
		{
			xyz(i, j, 0) = point_cloud(i, j).x;
			xyz(i, j, 1) = point_cloud(i, j).y;
			xyz(i, j, 2) = point_cloud(i, j).z;
		}
	}
	return xyz;
}

std::vector<float> as_ros_pose_msg(xt::xarray<float> h)
{
	float x = h(0, 3);
	float y = h(1, 3);
	float z = h(2, 3);
	float ox = 0.5 * std::copysign(1.0, h(2, 1) - h(1, 2)) * std::sqrt(h(0, 0) - h(1, 1) - h(2, 2) + 1);
	float oy = 0.5 * std::copysign(1.0, h(0, 2) - h(2, 0)) * std::sqrt(h(1, 1) - h(2, 2) - h(0, 0) + 1);
	float oz = 0.5 * std::copysign(1.0, h(1, 0) - h(0, 1)) * std::sqrt(h(2, 2) - h(0, 0) - h(1, 1) + 1);
	float w = 0.5 * std::sqrt(h(0, 0) + h(1, 1) + h(2, 2) + 1);
	return std::vector<float>({x, y, z, ox, oy, oz, w});
}
} //namespace CPE