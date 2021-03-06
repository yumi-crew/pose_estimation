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

#include <opencv2/core/utility.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/surface_matching/icp.hpp>

#include <vector>
#include <map>
#include <iostream>
#include <experimental/filesystem>
#include <fstream>

namespace pose_estimation{

class OpenCVSurfaceMatch
{
  public:
  OpenCVSurfaceMatch();
  ~OpenCVSurfaceMatch();

  void load_model(std::string model_path, int normals);
  void load_models_from_dir(std::string models_dir_path);
  void train_models();
  std::vector<float> find_object_in_scene(std::string object, cv::Mat& pc_scene);
  std::vector<std::string> get_trained_models();
  private:
  // std::vector<cv::Mat> models_; //store pc of models to identify
  std::vector<std::string> model_names_;
  std::map<std::string, cv::Mat> models_;
  std::string models_dir_path_;
  std::map<std::string, cv::ppf_match_3d::PPF3DDetector> detectors_; //detectors for each model, each key is the name of the model for the detector.
};

} //namespace
