#include "chessboard_pose_estimator.hpp"
#include <unistd.h>

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
  char *buf = getlogin();
  std::string u_name = buf;
  std::string path = "/home/" + u_name + "/abb_ws/src/object_files/ply";
  std::string scene = "/home/" + u_name + "/abb_ws/current_scene.ply";
  surface_match.load_models(path);
  std::cout << "after load\n";
  surface_match.update_current_scene(scene);
  std::cout << "after update\n";
  std::vector<float> pose1(7);
  bool success1 = surface_match.find_object_in_scene("screwdriver", pose1);

  std::cout << "success1 = " << success1 << std::endl;

  // for (auto p : pose)
  //   std::cout << p << " ";
  // std::cout << std::endl;
  return 0;
}
