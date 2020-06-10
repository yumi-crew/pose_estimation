// Copyright 2020 Markus Bjønnes and Marius Nilsen.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "halconcpp/HalconCpp.h"

#include <fstream>
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
  std::fstream pose_estimation_log_;
};
}