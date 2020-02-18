#include <chrono>
#include <memory>
#include <string>
#include <thread>

// lifecycle msgs and srvs
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"

// zivid-ros, pose_estimation srvs
#include "pose_estimation_interface/srv/estimate_pose.hpp"
#include "zivid_interfaces/srv/capture.hpp"
#include "zivid_interfaces/srv/camera_info_model_name.hpp"
#include "zivid_interfaces/srv/capture_assistant_suggest_settings.hpp"
#include "zivid_interfaces/srv/is_connected.hpp"

// lifecycle nodes to handle
static constexpr char const *zivid_camera_ls_node = "zivid_camera";
static constexpr char const *pose_estimation_ls_node = "pose_esimation";
// topic names for gettig and changing lifecycle node states
static constexpr char const *zc_get_state_topic = "zivid_camera/get_state";
static constexpr char const *pe_get_state_topic = "pose_estimation/get_state";
static constexpr char const *zc_change_state_topic = "zivid_camera/change_state";
static constexpr char const *pe_change_state_topic = "pose_estimation/change_state";

class ObjectPoseEstimator : public rclcpp::Node
{
public:
  ObjectPoseEstimator(const std::string &node_name);
  void init();
  unsigned int get_state(std::string ls_node, std::chrono::seconds time_out);
  bool change_state(std::string ls_node, std::uint8_t transition, std::chrono::seconds time_out);

  bool call_capture_srv(std::chrono::seconds time_out);
  bool call_estimate_pose_srv(std::chrono::seconds time_out);

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> zc_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> pe_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> zc_change_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> pe_change_state_;

  std::shared_ptr<rclcpp::Client<zivid_interfaces::srv::Capture>> client_ze_capture_;
  std::shared_ptr<rclcpp::Client<pose_estimation_interface::srv::EstimatePose>> client_pe_estimate_pose_;
};
