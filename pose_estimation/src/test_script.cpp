#include "chessboard_pose_estimator.hpp"

#include <Zivid/Zivid.h>
#include <Zivid/CloudVisualizer.h>

#include <opencv2/core/utility.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/surface_matching/icp.hpp>

#include "opencv_surface_match.hpp"

#include <iostream>
#include <vector>

int main()
{
	//chessboard estimator test
	// Zivid::Application zivid;
	// Zivid::Frame frame{"/home/markus/img01.zdf"};
	// Zivid::PointCloud point_cloud{frame.getPointCloud()};
	// xt::xarray<float> xyz = CPE::generate_xyz_xarray(point_cloud);
	// xt::xarray<int> rgb = CPE::generate_rgb_xarray(point_cloud);

	// CPE::ChessboardPoseEstimator pose_estimator;
	// pose_estimator.set_point_cloud(xyz, rgb);
	// bool found = pose_estimator.find_corners(20, 13);
	// pose_estimator.extract_feature_pnt_cld();

	// std::vector<float> h = pose_estimator.estimate_pose();
	// for(auto elem : h)
	// {
	// 	std::cout << elem << std::endl;
	// }

	// pose_estimator.show_img();

	// opencv surface_match test
	std::string model_file_name = "/home/markus/opencv/opencv_contrib/modules/surface_matching/samples/data/parasaurolophus_6700.ply";
	std::string scene_file_name = "/home/markus/opencv/opencv_contrib/modules/surface_matching/samples/data/rs1_normals.ply";
	cv::Mat pc_test = cv::ppf_match_3d::loadPLYSimple(scene_file_name.c_str(), 1);
	pose_estimation::OpenCVSurfaceMatch surface_match;
	//surface_match.load_models_from_dir("/home/markus/opencv/opencv_contrib/modules/surface_matching/samples/data/");
	surface_match.load_model(model_file_name, 1);
	surface_match.train_models();
	std::vector<float> pose = surface_match.find_object_in_scene("parasaurolophus_6700", pc_test);
	for(auto p:pose)
		std::cout << p << std::endl;

	// cv::Mat pc = cv::ppf_match_3d::loadPLYSimple(model_file_name.c_str(), 1);
	// // std::cout << pc << std::endl;
	// std::cout << "Training..." << std::endl;
	// int64 tick1 = cv::getTickCount();
	// cv::ppf_match_3d::PPF3DDetector detector(0.025, 0.05);
	// detector.trainModel(pc);
	// int64 tick2 = cv::getTickCount();
	// std::cout << std::endl
	// 					<< "Training complete in "
	// 					<< (double)(tick2 - tick1) / cv::getTickFrequency()
	// 					<< " sec" << std::endl
	// 					<< "Loading model..." << std::endl;

	// cv::Mat pc_test = cv::ppf_match_3d::loadPLYSimple(scene_file_name.c_str(), 1);

	// // Match the model to the scene and get the pose
	// std::cout << std::endl
	// 					<< "Starting matching..." << std::endl;
	// std::vector<cv::ppf_match_3d::Pose3DPtr> results;
	// tick1 = cv::getTickCount();
	// detector.match(pc_test, results, 1.0 / 40.0, 0.05);
	// tick2 = cv::getTickCount();
	// std::cout << std::endl
	// 					<< "PPF Elapsed Time "
	// 					<< (tick2 - tick1) / cv::getTickFrequency() << " sec" << std::endl;

	// // Get only first N results
	// int N = 2;
	// std::vector<cv::ppf_match_3d::Pose3DPtr> resultsSub(results.begin(), results.begin() + N);

	// // Create an instance of ICP
	// cv::ppf_match_3d::ICP icp(100, 0.005f, 2.5f, 8);
	// int64 t1 = cv::getTickCount();

	// // Register for all selected poses
	// std::cout << std::endl
	// 					<< "Performing ICP on " << N << " poses..." << std::endl;
	// icp.registerModelToScene(pc, pc_test, resultsSub);
	// int64 t2 = cv::getTickCount();

	// std::cout << std::endl
	// 					<< "ICP Elapsed Time "
	// 					<< (t2 - t1) / cv::getTickFrequency() << " sec" << std::endl;

	// std::cout << "Poses: " << std::endl;
	// // debug first five poses
	// for (size_t i = 0; i < resultsSub.size(); i++)
	// {
	// 	cv::ppf_match_3d::Pose3DPtr result = resultsSub[i];
	// 	std::cout << "Pose Result " << i << std::endl;
	// 	result->printPose();
	// 	if (i == 0)
	// 	{
	// 		cv::Mat pct = cv::ppf_match_3d::transformPCPose(pc, result->pose);
	// 		cv::ppf_match_3d::writePLY(pct, "para6700PCTrans0.ply");
	// 	}
	// 	if (i == 1)
	// 	{
	// 		cv::Mat pct = cv::ppf_match_3d::transformPCPose(pc, result->pose);
	// 		cv::ppf_match_3d::writePLY(pct, "para6700PCTrans1.ply");
	// 	}
	// }

	return 0;
}
