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
	xt::xarray<int> rgb = CPE::generate_rgb_xarray(point_cloud);

	CPE::ChessboardPoseEstimator pose_estimator;
	pose_estimator.set_point_cloud(xyz, rgb);
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
