#include "halconcpp/HalconCpp.h"

#include <map>
#include <string>
#include <iostream>
#include <vector>
#include <experimental/filesystem>

namespace pose_estimation
{
class HalconSurfaceMatch
{
  public:
  HalconSurfaceMatch();
  ~HalconSurfaceMatch();
  void load_models(std::string path_to_models_dir);
  void generate_surface_models();
  bool find_object_in_scene(std::string object, std::vector<float>& pose_estimate);
  void update_current_scene();
  void update_current_scene(std::string path_to_scene);

  private:
  std::map<std::string, HalconCpp::HObjectModel3D> models_;
  std::map<std::string, HalconCpp::HSurfaceModel> surface_models_;
  std::vector<std::string> model_names_;
  HalconCpp::HObjectModel3D current_scene_;
  std::string path_to_scene_;
  std::string models_dir_path_;
};
}