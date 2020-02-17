#include "object_pose_estimator.hpp"
#include <unistd.h>

using namespace std::chrono_literals;

template <typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT &future, WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do
  {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0))
    {
      break;
    }
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

ObjectPoseEstimator::ObjectPoseEstimator(const std::string &node_name) : Node(node_name)
{
}

void ObjectPoseEstimator::init()
{
  // might implement this functionality in the class constructor...
  zc_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(zc_get_state_topic);
  pe_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(pe_get_state_topic);
  zc_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(zc_change_state_topic);
  pe_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(pe_change_state_topic);
}

unsigned int ObjectPoseEstimator::get_state(std::string ls_node, std::chrono::seconds time_out = 3s)
{
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> node_get_state;

  if (!ls_node.compare("zivid_camera"))
  {
    node_get_state = zc_get_state_;
  }
  else if (!ls_node.compare("pose_estimation"))
  {
    node_get_state = pe_get_state_;
  }
  else
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Which node's state?");
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  if (!node_get_state->wait_for_service(time_out))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service" << node_get_state->get_service_name() << "is unavailable.");
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
  auto future_result = node_get_state->async_send_request(request);
  // auto future_status = wait_for_result(future_result, time_out);
  
  // if (future_status != std::future_status::ready)
  // {
  //   RCLCPP_ERROR_STREAM(get_logger(), "Time out while getting current state for node /" << ls_node);
  //   return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  // }


  auto future_status = future_result.wait_for(3s);

  if (true)
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
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> node_change_state;

  if (!ls_node.compare("zivid_camera"))
  {
    node_change_state = zc_change_state_;
  }
  else if (!ls_node.compare("pose_estimation"))
  {
    node_change_state = pe_change_state_;
  }
  else
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Change which node's state?");
  }

  if (!node_change_state->wait_for_service(time_out))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service " << node_change_state->get_service_name() << " is unavailable.");
    return false;
  }

  auto future_result = node_change_state->async_send_request(request);
  // auto future_status = wait_for_result(future_result, time_out);
  auto future_status = future_result.wait_for(3s);
  // if (future_status != std::future_status::ready)
  // {
  //   RCLCPP_ERROR_STREAM(get_logger(), "Service timout while changing state for " << ls_node);
  //   return false;
  // }

  if (true)
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
