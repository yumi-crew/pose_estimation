#include "opencv_surface_match.hpp"

namespace pose_estimation
{

OpenCVSurfaceMatch::OpenCVSurfaceMatch()
{
}

OpenCVSurfaceMatch::~OpenCVSurfaceMatch()
{
}

void OpenCVSurfaceMatch::load_model(std::string model_path, int normals)
{
  std::cout << "load_model: "
            << model_path << ", ";
  std::string name = model_path.substr(model_path.find_last_of("/") + 1);
  name = name.substr(0, name.find("."));
  std::cout << "as: " << name << std::endl;
  model_names_.push_back(name);
  models_[name] = cv::ppf_match_3d::loadPLYSimple(model_path.c_str(), normals);
}

void OpenCVSurfaceMatch::load_models_from_dir(std::string models_dir_path)
{
  models_dir_path_ = models_dir_path;
  std::string extension;
  for (const auto &entry : std::experimental::filesystem::directory_iterator(models_dir_path))
  {
    extension = entry.path().string().substr(entry.path().string().find(".") + 1, 3);
    if (extension.compare("ply") == 0)
      load_model(entry.path().string(), 1);
  }
}

//todo: if trained model already exists deserialize it instead of training a new model. If it does not train model and save for later.
void OpenCVSurfaceMatch::train_models()
{
  int64 tick1, tick2;
  for (uint i = 0; i < model_names_.size(); ++i)
  {
    std::cout << "Training model: " << model_names_[i] << std::endl;
    tick1 = cv::getTickCount();
    detectors_[model_names_[i]] = cv::ppf_match_3d::PPF3DDetector(0.03, 0.05, 30); //tune params later
    detectors_[model_names_[i]].trainModel(models_[model_names_[i]]);
    tick2 = cv::getTickCount();
    std::cout << "Training complete in "
              << (double)(tick2 - tick1) / cv::getTickFrequency()
              << " sec" << std::endl;
  }
  // for (uint i = 0; i < model_names_.size(); ++i)
  // {
  //   std::string trained_model_file_name = models_dir_path_ + "/" + model_names_[i] + ".xml";
  //   std::ifstream trained_model_file(trained_model_file_name);
  //   if (!trained_model_file.good())
  //   {
  //     std::cout << "Training model: " << model_names_[i] << std::endl;
  //     tick1 = cv::getTickCount();
  //     detectors_[model_names_[i]] = cv::ppf_match_3d::PPF3DDetector(0.05, 0.05, 30); //tune params later
  //     detectors_[model_names_[i]].trainModel(models_[model_names_[i]]);
  //     tick2 = cv::getTickCount();
  //     std::cout << "Training complete in "
  //               << (double)(tick2 - tick1) / cv::getTickFrequency()
  //               << " sec" << std::endl;
  //     std::cout << "Saving trained model" << std::endl;
  //     tick1 = cv::getTickCount();
  //     cv::FileStorage fs_out(trained_model_file_name, cv::FileStorage::WRITE);
  //     detectors_[model_names_[i]].write(fs_out);
  //     fs_out.release();
  //     tick2 = cv::getTickCount();
  //     std::cout << "Model saved in "
  //               << (double)(tick2 - tick1) / cv::getTickFrequency()
  //               << " sec" << std::endl;
  //   }
  //   else
  //   {
  //     trained_model_file.close();
  //     std::cout << "Found pretrained model for " << model_names_[i] << std::endl;
  //     tick1 = cv::getTickCount();
  //     cv::FileStorage fs_load(trained_model_file_name, cv::FileStorage::READ);
  //     std::cout << "filestorage ok" << std::endl;
  //     // cv::ppf_match_3d::PPF3DDetector det;
  //     // det.read(fs_load.root());
  //     detectors_[model_names_[i]] = cv::ppf_match_3d::PPF3DDetector();
  //     std::cout << "detector init" << std::endl;
  //     detectors_[model_names_[i]].read(fs_load.root());
  //     std::cout << "after read" << std::endl;
  //     fs_load.release();
  //     // detectors_[model_names_[i]] = det;
  //     tick2 = cv::getTickCount();
  //     std::cout << "Loaded pretrained model for "
  //               << model_names_[i] << " in "
  //               << (double)(tick2 - tick1) / cv::getTickFrequency()
  //               << " sec" << std::endl;
  //   }
  // }
}

std::vector<float> OpenCVSurfaceMatch::find_object_in_scene(std::string object, cv::Mat &pc_scene)
{
  std::cout << "Finding object: " << object << std::endl;
  std::cout << "Calculating normals" << std::endl;
  cv::Vec3d viewpoint(0, 0, 0);
  cv::Mat pc_scene_normals;
  cv::ppf_match_3d::computeNormalsPC3d(pc_scene, pc_scene_normals, 10, true, viewpoint);
  try
  {
    int64 tick1 = cv::getTickCount();
    std::vector<cv::ppf_match_3d::Pose3DPtr> results;
    std::cout << "before match\n";
    detectors_[object].match(pc_scene_normals, results, 1.0 / 1.0, 0.025);
    std::cout << "before icp\n";
    for (int i = 0; i < results.size(); ++i)
    {
      // std::cout << results[i]->q << std::endl;
      double q0 = results[i]->q[0];
      double q1 = results[i]->q[1];
      double q2 = results[i]->q[2];
      double q3 = results[i]->q[3];
      double length = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
      q0 /= length;
      q1 /= length;
      q2 /= length;
      q3 /= length;
      // std::cout << q0 << " " << q1 << " " << q2 << " " << q3 << std::endl;
      cv::Vec4d new_q = {q0, q1, q2, q3};
      cv::Vec3d new_t = {results[i]->t[0], results[i]->t[1], results[i]->t[2]};
      results[i]->updatePoseQuat(new_q, new_t);
      // results[i]->printPose();
    }
    cv::ppf_match_3d::ICP icp(250, 0.005f, 2.5f, 8);
    std::cout << std::endl
              << results.size() << std::endl;

    int subset_size{results.size()};
    if (results.size() > 15)
      subset_size = 15;
    std::vector<cv::ppf_match_3d::Pose3DPtr> results_subset(results.begin(), results.begin() + subset_size);

    icp.registerModelToScene(models_[object], pc_scene_normals, results_subset);
    int64 tick2 = cv::getTickCount();
    std::cout << "Matching complete in "
              << (double)(tick2 - tick1) / cv::getTickFrequency()
              << " sec" << std::endl;

    // identify the match with the largest number of votes

    int max_votes{0};
    int best_ptm_idx{0};
    float min_res{10000.0};
    uint best_idx{0};
    for (uint i = 0; i < results_subset.size(); ++i)
    {
      std::cout << "pose " << i << ": " << std::endl;
      results_subset[i]->printPose();
      if (results_subset[i]->numVotes > max_votes)
      {
        max_votes = results_subset[i]->numVotes;
        best_idx = i;
      }
      if (results.size() > 5)
      {
        if (results_subset[i]->residual < min_res && results_subset[i]->numVotes > 400)
        {
          min_res = results_subset[i]->residual;
          best_idx = i;
        }
      }
    }
    std::cout << "best pose: " << best_idx << ", max votes:  = " << max_votes << std::endl;

    // save the model and scene as ply files to review the matching
    for (int i = 0; i < results_subset.size(); ++i)
    {
      std::string name = std::to_string(i) + ".ply";
      cv::ppf_match_3d::writePLY(cv::ppf_match_3d::transformPCPose(models_[object], results_subset[i]->pose), name.c_str());
    }
    cv::ppf_match_3d::writePLY(pc_scene, "scene.ply");
    std::vector<float> pose(7);
    for (int i = 0; i < 3; ++i)
      pose[i] = results[best_idx]->t[i];
    // for (int i = 0; i < 4; ++i)
    //   pose[i + 3] = results[best_idx]->q[i];
    pose[3] = results[best_idx]->q[1];
    pose[4] = results[best_idx]->q[2];
    pose[6] = results[best_idx]->q[3];
    pose[6] = results[best_idx]->q[0];
    return pose;
  }
  catch (std::exception &e)
  {
    std::cout << e.what();
  }
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

} // namespace pose_estimation