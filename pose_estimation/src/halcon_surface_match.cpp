#include "halcon_surface_match.hpp"
#include <unistd.h>

namespace pose_estimation
{
HalconSurfaceMatch::HalconSurfaceMatch()
{
  current_scene_ = HalconCpp::HObjectModel3D();
  char *buf = getlogin();
  std::string u_name = buf;
  path_to_scene_ = "/home/" + u_name + "/abb_ws/current_scene.ply";
}

HalconSurfaceMatch::~HalconSurfaceMatch()
{
}

void HalconSurfaceMatch::load_models(std::string path_to_models_dir)
{
  models_dir_path_ = path_to_models_dir;
  std::string extension;
  std::string model_path;
  for (const auto &entry : std::experimental::filesystem::directory_iterator(models_dir_path_))
  {
    extension = entry.path().string().substr(entry.path().string().find(".") + 1, 3);
    if (extension.compare("ply") == 0)
    {
      model_path = entry.path().string();
      std::cout << "load_model: "
                << model_path << ", ";
      std::string name = model_path.substr(model_path.find_last_of("/") + 1);
      name = name.substr(0, name.find("."));
      std::cout << "as: " << name << std::endl;
      model_names_.push_back(name);
      HalconCpp::HTuple gen_param_name, gen_param_value, object_model, status;
      models_[name].ReadObjectModel3d(model_path.c_str(), "m", gen_param_name, gen_param_value);
    }
  }
  // create surface models (training stage)
  for (auto name : model_names_)
  {
    HalconCpp::HTuple gen_param_name, gen_param_value;
    surface_models_[name] = models_[name].CreateSurfaceModel(0.03, gen_param_name, gen_param_value);
  }
}

void HalconSurfaceMatch::update_current_scene()
{
  HalconCpp::HTuple gen_param_name, gen_param_value, status;
  HalconCpp::HObjectModel3D scene_without_normals;

  HalconCpp::ClearObjectModel3d(current_scene_);
  scene_without_normals.ReadObjectModel3d(path_to_scene_.c_str(), "m", gen_param_name, gen_param_value);
  try
  {
    current_scene_ = scene_without_normals.SurfaceNormalsObjectModel3d("mls", gen_param_name, gen_param_value);
  }
  catch (HalconCpp::HException &e)
  {
    std::cout << e.ErrorMessage() << std::endl;
  }
}

void HalconSurfaceMatch::update_current_scene(std::string path_to_scene)
{
  path_to_scene_ = path_to_scene;
  update_current_scene();
}

bool HalconSurfaceMatch::find_object_in_scene(std::string object, std::vector<float> &pose_estimate)
{
  HalconCpp::HTuple gen_param_name, gen_param_value, status, score, result_id;
  HalconCpp::HSurfaceMatchingResult result;
  HalconCpp::HTuple pose;

  gen_param_name[0] = "num_matches";
  gen_param_value[0] = 10;
  gen_param_name[1] = "pose_ref_use_scene_normals";
  gen_param_value[1] = "true";
  gen_param_name[2] = "dense_pose_refinement";
  gen_param_value[2] = "true";
  gen_param_name[3] = "pose_ref_num_steps";
  gen_param_value[3] = 20;
  

  try
  {
    pose = current_scene_.FindSurfaceModel(surface_models_[object], 0.03, 1.0, 0, "true", gen_param_name, gen_param_value, &score, &result);

    std::cout << pose.Type() << std::endl;

    HalconCpp::HTuple quat;
    HalconCpp::PoseToQuat(pose, &quat);
    for (int i = 0; i < 3; ++i)
      pose_estimate[i] = pose[i];

    // the first element of quat is the real part of the quaternion
    pose_estimate[3] = quat[1];
    pose_estimate[4] = quat[2];
    pose_estimate[5] = quat[3];
    pose_estimate[6] = quat[0];

    std::cout << "POSE TO STRING****************" << std::endl;
    std::cout << pose.ToString() << std::endl;

    //check score for success.
    std::cout << "matching score: " << score.ToString() << std::endl;
  }
  catch (HalconCpp::HException &e)
  {
    std::cout << e.ErrorMessage() << std::endl;
    return false;
  }
  if (static_cast<double>(score) < 0.20)
    return false;
  return true;
}
} // namespace pose_estimation