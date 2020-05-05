#include "halcon_surface_match.hpp"
#include <unistd.h>

namespace pose_estimation
{
HalconSurfaceMatch::HalconSurfaceMatch()
{
  current_scene_ = HalconCpp::HObjectModel3D();
  char buf[20];
  getlogin_r(buf, 20);
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
      HalconCpp::HTuple genParamName, genParamValue, object_model, status;
      models_[name].ReadObjectModel3d(model_path.c_str(), "m", genParamName, genParamValue);
    }
  }
  // create surface models (training stage)
  for (auto name : model_names_)
  {
    HalconCpp::HTuple genParamName, genParamValue;
    surface_models_[name] = models_[name].CreateSurfaceModel(0.02, genParamName, genParamValue);
  }
}

void HalconSurfaceMatch::update_current_scene()
{
  HalconCpp::HTuple genParamName, genParamValue, status;
  HalconCpp::HObjectModel3D scene_without_normals;

  HalconCpp::ClearObjectModel3d(current_scene_);
  scene_without_normals.ReadObjectModel3d(path_to_scene_.c_str(), "m", genParamName, genParamValue);

  current_scene_ = scene_without_normals.SurfaceNormalsObjectModel3d("mls", genParamName, genParamValue);
}

std::vector<float> HalconSurfaceMatch::find_object_in_scene(std::string object)
{
  HalconCpp::HTuple genParamName, genParamValue, status, score, result_id;
  HalconCpp::HSurfaceMatchingResult result;
  HalconCpp::HTuple pose;
  try
  {
    pose = current_scene_.FindSurfaceModel(surface_models_[object], 0.03, 0.5, 0.0, "true", genParamName, genParamValue, &score, &result);
  }
  catch (HalconCpp::HException &e)
  {
    std::cout << e.ErrorMessage();
  }
  HalconCpp::HTuple quat;
  HalconCpp::PoseToQuat(pose, &quat);
  std::vector<float> pose_vec(7);
  for (int i = 0; i < 3; ++i)
    pose_vec[i] = pose[i];

  // the first element of quat is the real part of the quaternion
  pose_vec[3] = quat[1];
  pose_vec[4] = quat[2];
  pose_vec[5] = quat[3];
  pose_vec[6] = quat[0];
  return pose_vec;
}
} // namespace pose_estimation