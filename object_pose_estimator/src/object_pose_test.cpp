#include "object_pose_estimator.hpp"
#include <unistd.h>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto pose_estimation_manager = std::make_shared<ObjectPoseEstimator>("object_pose_estimator");
  pose_estimation_manager->init();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(pose_estimation_manager);

  auto state1 = pose_estimation_manager->get_state("zivid_camera", 3s);
  auto state2 = pose_estimation_manager->get_state("pose_estimation", 3s);

  auto transition_success1 = pose_estimation_manager->change_state(
      "zivid_camera", lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, 5s);
  auto transition_success2 = pose_estimation_manager->change_state(
      "pose_estimation", lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, 5s);

  auto state3 = pose_estimation_manager->get_state("zivid_camera", 3s);
  auto state4 = pose_estimation_manager->get_state("pose_estimation", 3s);

  auto transition_success3 = pose_estimation_manager->change_state(
      "zivid_camera", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 10s);
  auto transition_success4 = pose_estimation_manager->change_state(
      "pose_estimation", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 5s);
  auto state5 = pose_estimation_manager->get_state("zivid_camera", 3s);
  auto state6 = pose_estimation_manager->get_state("pose_estimation", 3s);

  bool cap_success{false};
  bool est_success{false};

  while (rclcpp::ok)
  {
    cap_success = pose_estimation_manager->call_capture_srv(3s);
    est_success = pose_estimation_manager->call_estimate_pose_srv(3s);
  }

  rclcpp::shutdown();
  return 0;
}