#include "chessboard_pose_estimator.h"

#include <Zivid/Zivid.h>
#include <Zivid/CloudVisualizer.h>


#include <iostream>

int main()
{
	Zivid::Application zivid;
	Zivid::Frame frame{"/home/markus/img01.zdf"};

	Zivid::PointCloud point_cloud{frame.getPointCloud()};
	xt::xarray<float> xyz = CPE::generate_xyz_xarray(point_cloud);
	cv::Mat rgb = CPE::generate_cv_img(point_cloud);

	CPE::ChessboardPoseEstimator pose_estimator{rgb, xyz};
	pose_estimator.find_corners();
	pose_estimator.extract_feature_pnt_cld();

	xt::xarray<float> h = CPE::plane_fit(pose_estimator.feature_pnt_cld_); 

	pose_estimator.show_img();

	return 0;
}
