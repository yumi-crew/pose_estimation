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

class ObjectPoseEstimator : public rclcpp::Node
{
public:
  ObjectPoseEstimator(const std::string &node_name);
  unsigned int get_state(std::string ls_node, std::chrono::seconds time_out);
  bool change_state(std::string ls_node, std::uint8_t transition, std::chrono::seconds time_out);

  bool call_capture_srv(std::chrono::seconds time_out);
  bool call_estimate_pose_srv(std::chrono::seconds time_out);

private:

};
