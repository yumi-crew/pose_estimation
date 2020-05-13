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
  /**
   * Returns the state of a lifecycle node. 
   * @param ls_node name of the lifecycle node for which the state is returned.
   * @param time_out time out for the service call.
   * @returns integer linked to the state of the lifecycle node, enumerated by lifecycle_msgs::msg::State.
  **/ 
  unsigned int get_state(std::string ls_node, std::chrono::seconds time_out);

  /**
   * Changes the state of a lifecycle node.
   * @param ls_node name of the lifecycle node.
   * @param transition transition to be triggered, see lifecycle_msgs::msg::Transition.
   * @param time_out time out for the service call.
   * @returns true if transition successful.
  **/ 
  bool change_state(std::string ls_node, std::uint8_t transition, std::chrono::seconds time_out);

  /**
   * Calls the capture service of the zivid_camera node. Results in a pointcloud2 being published to /points.
   * @returns true if successful.
  **/ 
  bool call_capture_srv(std::chrono::seconds time_out);

  /**
   * Calls the pose estimation service of the pose_estimation node.
   * @param object name of object to be found.
   * @param num_planes number of planes to remove from point cloud. Useful for removing planar backgrouds. Results in faster matching.
   * @param time_out time out for the service call.
   * @param filter_out alternatives are "outliers" which removes points outside a sphere, or "inliers" which removes points inside a shpere.
   * @param filter_radius radius of the filter sphere.
   * @param store_filter_pose stores the pose found as the new filter point. Center of the filter sphere.
   * @returns true if a pose for the requested object was found.
  **/ 
  bool call_estimate_pose_srv(std::string object, int num_planes, std::chrono::seconds time_out, std::string filter_out = "", float filter_radius = 0.0, bool store_filter_pose = false);

  /**
   * Calls the service for initializing OpenCV surface match functionality.
   * @param model_dir_path the path to a directory containing .ply model files.
   * @param time_out time out for the service call.
   * @returns true if initializaton was successful.
  **/
  bool call_init_cv_surface_match_srv(std::string model_dir_path, std::chrono::seconds time_out);

  /**
   * Calls the service for initializing Halcon surface match functionality.
   * @param model_dir_path the path to a directory containing .ply model files.
   * @param time_out time out for the service call.
   * @returns true if initializaton was successful.
  **/
  bool call_init_halcon_surface_match_srv(std::string model_dir_path, std::chrono::seconds time_out);

  /**
   * Calls service for setting parameters stored in the camera_parameters_ member variable to the /zivid_camera node
   * @param time_out time out for the service call.
  **/
  bool call_set_param_srv(std::chrono::seconds time_out);

  /**
   * Adds a camera parameter to camera_parameters_. 
   * @param name camera parameter name. See the bottom of zivid_camera.cpp in the zivid_ros module for options. 
   * @param value value of camera parameter.
  **/
  void add_camera_parameter(const std::string &name, const rclcpp::ParameterValue &value);

  /**
   * Clears the contents of camera_parameters_
  **/
  void clear_camera_parameters();

  std::shared_ptr<PoseTransformer> pose_transformer;

private:
  std::vector<rclcpp::Parameter> camera_parameters_;
};
