#include "chessboard_pose_estimator.hpp"

#include <Zivid/Zivid.h>
#include <Zivid/CloudVisualizer.h>

#include <opencv2/core/utility.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/surface_matching/icp.hpp>

#include "opencv_surface_match.hpp"
#include "halcon_surface_match.hpp"

#include <iostream>
#include <vector>

int main()
{
	pose_estimation::HalconSurfaceMatch surface_match;
	std::string path = "/home/markus/Documents/models_ply/";
	surface_match.load_models(path);
	std::cout << "after load\n";
	surface_match.update_current_scene();
	std::cout << "after update\n";
	std::vector<float> pose = surface_match.find_object_in_scene("screwdriver");
	for(auto p:pose)
		std::cout << p << " ";
	std::cout << "\npossible success!" << std::endl;
	return 0;
}
