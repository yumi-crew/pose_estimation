#include "object_pose_estimator.hpp"
#include <unistd.h>

using namespace std::chrono_literals;

ObjectPoseEstimator::ObjectPoseEstimator(const std::string &node_name) : Node(node_name)
{
}

unsigned int ObjectPoseEstimator::get_state(std::string ls_node, std::chrono::seconds time_out = 3s)
{
  auto temp_node = std::make_unique<rclcpp::Node>("temp_node");
  auto temp_node_client = temp_node->create_client<lifecycle_msgs::srv::GetState>(ls_node + "/get_state");
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

  if (!temp_node_client->wait_for_service(10s))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service " << temp_node_client->get_service_name() << " is unavailable.");
    return false;
  }
  auto future_result = temp_node_client->async_send_request(request);
  auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), future_result, time_out);

  if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service timout while getting state for " << ls_node);
    return false;
  }
  if (future_result.get())
  {
    RCLCPP_INFO_STREAM(get_logger(),
                       "Node /" << ls_node << " has current state: " << future_result.get()->current_state.label.c_str());
    return future_result.get()->current_state.id;
  }
  else
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to get current state of node /" << ls_node);
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
}

bool ObjectPoseEstimator::change_state(std::string ls_node, std::uint8_t transition, std::chrono::seconds time_out = 8s)
{
  auto temp_node = std::make_unique<rclcpp::Node>("temp_node");
  auto temp_node_client = temp_node->create_client<lifecycle_msgs::srv::ChangeState>(ls_node + "/change_state");

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  // std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state;

  if (!temp_node_client->wait_for_service(10s))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service " << temp_node_client->get_service_name() << " is unavailable.");
    return false;
  }

  auto future_result = temp_node_client->async_send_request(request);
  auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), future_result, time_out);

  if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service timout while changing state for " << ls_node);
    return false;
  }

  if (future_result.get()->success)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Transition " << static_cast<int>(transition) << " successfully triggered.");
    return true;
  }
  else
  {
    RCLCPP_WARN_STREAM(get_logger(), "Failed to trigger transition " << static_cast<unsigned int>(transition));
    return false;
  }
}

bool ObjectPoseEstimator::call_capture_srv(std::chrono::seconds time_out = 5s)
{
  auto temp_node = std::make_unique<rclcpp::Node>("temp_node");
  auto temp_node_client = temp_node->create_client<zivid_interfaces::srv::Capture>("/capture");
  auto request = std::make_shared<zivid_interfaces::srv::Capture::Request>();

  if (!temp_node_client->wait_for_service(10s))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service " << temp_node_client->get_service_name() << " is unavailable.");
    return false;
  }
  auto future_result = temp_node_client->async_send_request(request);
  auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), future_result, time_out);

  if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service timout while capturing frame");
    return false;
  }
  return true;
}

bool ObjectPoseEstimator::call_estimate_pose_srv(std::chrono::seconds time_out = 5s)
{
  auto temp_node = std::make_unique<rclcpp::Node>("temp_node");
  auto temp_node_client = temp_node->create_client<pose_estimation_interface::srv::EstimatePose>("/estimate_pose");
  auto request = std::make_shared<pose_estimation_interface::srv::EstimatePose::Request>();

  if (!temp_node_client->wait_for_service(10s))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service " << temp_node_client->get_service_name() << " is unavailable.");
    return false;
  }

  auto future_result = temp_node_client->async_send_request(request);
  auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), future_result, time_out);

  if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service timout while estimating pose");
    return false;
  }
  if (!future_result.get()->success)
  {
    return false;
  }
  return true;
}