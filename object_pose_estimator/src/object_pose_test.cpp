#include "object_pose_estimator.hpp"
#include <unistd.h>
#include <signal.h>

using namespace std::chrono_literals;

std::shared_ptr<ObjectPoseEstimator> pose_estimation_manager;

void signal_callback_handler(int signum)
{
  std::cout << "Caught signal " << signum << std::endl;
  pose_estimation_manager->change_state(
      "zivid_camera", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 10s);
  pose_estimation_manager->change_state(
      "pose_estimation", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 10s);
  pose_estimation_manager->change_state(
      "zivid_camera", lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN, 10s);
  pose_estimation_manager->change_state(
      "pose_estimation", lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN, 10s);
  rclcpp::shutdown();
  exit(signum);
}

int main(int argc, char **argv)
{
  // Ctrl+C handler
  signal(SIGINT, signal_callback_handler);

  rclcpp::init(argc, argv);
  pose_estimation_manager = std::make_shared<ObjectPoseEstimator>("object_pose_estimator");
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
    cap_success = pose_estimation_manager->call_capture_srv(10s);
    est_success = pose_estimation_manager->call_estimate_pose_srv(10s);
  }

  rclcpp::shutdown();
  return 0;
}