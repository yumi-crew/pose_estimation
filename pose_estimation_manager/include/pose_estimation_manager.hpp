#include <chrono>
#include <memory>
#include <string>
#include <thread>

// lifecycle msgs and srvs
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rcl_interfaces/srv/set_parameters.hpp"

#include "rclcpp/rclcpp.hpp"

// zivid-ros, pose_estimation srvs
#include "pose_estimation_interface/srv/estimate_pose.hpp"
#include "pose_estimation_interface/srv/init_cv_surface_match.hpp"
#include "pose_estimation_interface/srv/init_halcon_surface_match.hpp"
#include "zivid_interfaces/srv/capture.hpp"
#include "zivid_interfaces/srv/camera_info_model_name.hpp"
#include "zivid_interfaces/srv/capture_assistant_suggest_settings.hpp"
#include "zivid_interfaces/srv/is_connected.hpp"
#include "pose_transformer.hpp"

class PoseEstimationManager : public rclcpp::Node
{
public:
  PoseEstimationManager(const std::string &node_name);
  unsigned int get_state(std::string ls_node, std::chrono::seconds time_out);
  bool change_state(std::string ls_node, std::uint8_t transition, std::chrono::seconds time_out);

  bool call_capture_srv(std::chrono::seconds time_out);
  bool call_estimate_pose_srv(std::string object, std::chrono::seconds time_out);
  bool call_init_cv_surface_match_srv(std::string model_dir_path, int num_planes, std::chrono::seconds time_out);
  bool call_init_halcon_surface_match_srv(std::string model_dir_path, int num_planes, std::chrono::seconds time_out);
  bool call_set_param_srv(std::chrono::seconds time_out);

  void add_camera_parameter(const std::string &name, const rclcpp::ParameterValue &value);
  void clear_camera_parameters();

  std::shared_ptr<PoseTransformer> pose_transformer;
private:

  std::vector<rclcpp::Parameter> camera_parameters_;

};
