#include "chessboard_pose_estimator.hpp"

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

	std::vector<float> h = pose_estimator.estimate_pose(); 
	for(auto elem : h)
	{
		std::cout << elem << std::endl;
	}
	
	pose_estimator.show_img();

	return 0;
}
