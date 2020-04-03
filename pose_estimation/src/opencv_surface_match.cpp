#include "opencv_surface_match.hpp"

namespace pose_estimation{

OpenCVSurfaceMatch::OpenCVSurfaceMatch()
{
}

OpenCVSurfaceMatch::~OpenCVSurfaceMatch()
{
}

void OpenCVSurfaceMatch::load_model(std::string model_path, int normals)
{
  std::cout << "load_model: "
            << model_path << std::endl;
  std::string name = model_path.substr(model_path.find_last_of("/") + 1);
  name = name.substr(0, name.find("."));
  std::cout << "as: " << name << std::endl;
  model_names_.push_back(name);
  models_[name] = cv::ppf_match_3d::loadPLYSimple(model_path.c_str(), normals);
}

void OpenCVSurfaceMatch::load_models_from_dir(std::string models_dir_path)
{
  for(const auto &entry : std::experimental::filesystem::directory_iterator(models_dir_path))
  {
    load_model(entry.path().string(), 1);
  }
}

void OpenCVSurfaceMatch::train_models()
{
  for (uint i = 0; i < model_names_.size(); ++i)
  {
    std::cout << "Training model: " << model_names_[i] << std::endl;
    int64 tick1 = cv::getTickCount();
    detectors_[model_names_[i]] = cv::ppf_match_3d::PPF3DDetector(0.025, 0.05); //tune params later
    detectors_[model_names_[i]].trainModel(models_[model_names_[i]]);
    int64 tick2 = cv::getTickCount();
    std::cout << "Training complete in "
              << (double)(tick2 - tick1) / cv::getTickFrequency()
              << " sec" << std::endl;
  }
}

std::vector<float> OpenCVSurfaceMatch::find_object_in_scene(std::string object, cv::Mat& pc_scene)
{
  std::cout << "Finding object: " << object << std::endl;
  int64 tick1 = cv::getTickCount();
  std::vector<cv::ppf_match_3d::Pose3DPtr> results;
  detectors_[object].match(pc_scene, results, 1.0 / 40.0, 0.05);
  cv::ppf_match_3d::ICP icp(100, 0.005f, 2.5f, 8);

  std::vector<cv::ppf_match_3d::Pose3DPtr> results_subset(results.begin(), results.begin() + 3);

  icp.registerModelToScene(models_[object], pc_scene, results_subset);
  int64 tick2 = cv::getTickCount();
  std::cout << "Matching complete in "
            << (double)(tick2 - tick1) / cv::getTickFrequency()
            << " sec" << std::endl;

  // identify the match with the lowest residual (error)
  double min_res{results_subset[0]->residual};
  uint best_idx{0};
  for (uint i = 0; i < results_subset.size(); ++i)
  {
    if (results_subset[i]->residual < min_res)
    {
      min_res = results_subset[i]->residual;
      best_idx = i;
    }
  }
  std::cout << "best pose: " << best_idx << ", residual = " << min_res << std::endl;

  std::vector<float> pose(7);
  for (int i = 0; i < 3; ++i)
    pose[i] = results_subset[best_idx]->t[i];
  for (int i = 3; i < 7; ++i)
    pose[i] = results_subset[best_idx]->q[i];
  return pose;
}

std::vector<std::string> OpenCVSurfaceMatch::get_trained_models()
{
  std::vector<std::string> trained_models;
  for (auto i = detectors_.begin(); i != detectors_.end(); ++i)
  {
    trained_models.push_back(i->first);
  }
  return trained_models;
}

} //namespace