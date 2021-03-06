#include "pose_estimation_manager.hpp"
#include <unistd.h>

using namespace std::chrono_literals;

PoseEstimationManager::PoseEstimationManager(const std::string &node_name)
    : Node(node_name),
      pose_transformer{std::make_shared<PoseTransformer>()},
      camera_parameters_{std::vector<rclcpp::Parameter>()}
{
}

unsigned int PoseEstimationManager::get_state(std::string ls_node, std::chrono::seconds time_out)
{
  auto temp_node{std::make_unique<rclcpp::Node>("temp_node")};
  auto temp_node_client{temp_node->create_client<lifecycle_msgs::srv::GetState>(ls_node + "/get_state")};
  auto request{std::make_shared<lifecycle_msgs::srv::GetState::Request>()};

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

bool PoseEstimationManager::change_state(std::string ls_node, std::uint8_t transition, std::chrono::seconds time_out)
{
  auto temp_node{std::make_unique<rclcpp::Node>("temp_node")};
  auto temp_node_client{temp_node->create_client<lifecycle_msgs::srv::ChangeState>(ls_node + "/change_state")};
  auto request{std::make_shared<lifecycle_msgs::srv::ChangeState::Request>()};
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

bool PoseEstimationManager::call_capture_srv(std::chrono::seconds time_out)
{
  auto temp_node{std::make_unique<rclcpp::Node>("temp_node")};
  auto temp_node_client{temp_node->create_client<zivid_interfaces::srv::Capture>("/capture")};
  auto request{std::make_shared<zivid_interfaces::srv::Capture::Request>()};

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

bool PoseEstimationManager::call_estimate_pose_srv(std::string object, int num_planes, std::chrono::seconds time_out, std::string filter_out, float filter_radius, bool store_filter_pose)
{
  auto temp_node{std::make_unique<rclcpp::Node>("temp_node")};
  auto temp_node_client{temp_node->create_client<pose_estimation_interface::srv::EstimatePose>("/estimate_pose")};
  auto request{std::make_shared<pose_estimation_interface::srv::EstimatePose::Request>()};
  request->object = object;
  request->num_planes = num_planes;
  request->filter_out = filter_out;
  request->filter_radius = filter_radius;
  request->store_filter_pose = store_filter_pose;

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

bool PoseEstimationManager::call_init_cv_surface_match_srv(std::string model_dir_path, std::chrono::seconds time_out)
{
  auto temp_node{std::make_unique<rclcpp::Node>("temp_node")};
  auto temp_node_client{temp_node->create_client<pose_estimation_interface::srv::InitCvSurfaceMatch>("/init_cv_surface_match")};
  auto request{std::make_shared<pose_estimation_interface::srv::InitCvSurfaceMatch::Request>()};
  request->model_dir_path = model_dir_path;

  if (!temp_node_client->wait_for_service(10s))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service " << temp_node_client->get_service_name() << " is unavailable.");
    return false;
  }

  auto future_result = temp_node_client->async_send_request(request);
  auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), future_result, time_out);

  if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service timout while initializig surface match");
    return false;
  }
  if (!future_result.get()->success)
  {
    return false;
  }
  return true;
}

bool PoseEstimationManager::call_init_halcon_surface_match_srv(std::string model_dir_path, std::chrono::seconds time_out)
{
  auto temp_node{std::make_unique<rclcpp::Node>("temp_node")};
  auto temp_node_client{temp_node->create_client<pose_estimation_interface::srv::InitHalconSurfaceMatch>("/init_halcon_surface_match")};
  auto request{std::make_shared<pose_estimation_interface::srv::InitHalconSurfaceMatch::Request>()};
  request->model_dir_path = model_dir_path;

  if (!temp_node_client->wait_for_service(10s))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service " << temp_node_client->get_service_name() << " is unavailable.");
    return false;
  }

  auto future_result = temp_node_client->async_send_request(request);
  auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), future_result, time_out);

  if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service timout while initializig surface match");
    return false;
  }
  if (!future_result.get()->success)
  {
    return false;
  }
  return true;
}

bool PoseEstimationManager::call_set_param_srv(std::chrono::seconds time_out)
{
  auto temp_node{std::make_unique<rclcpp::Node>("temp_node")};
  auto temp_node_client{temp_node->create_client<rcl_interfaces::srv::SetParameters>("zivid_parameter_server/set_parameters")};
  auto request{std::make_shared<rcl_interfaces::srv::SetParameters::Request>()};

  std::transform(camera_parameters_.begin(), camera_parameters_.end(), std::back_inserter(request->parameters),
                 [](rclcpp::Parameter p) { return p.to_parameter_msg(); });

  if (!temp_node_client->wait_for_service(10s))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service " << temp_node_client->get_service_name() << " is unavailable.");
    return false;
  }

  auto future_result = temp_node_client->async_send_request(request);
  auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), future_result, time_out);

  if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Service timout while setting camera parameters");
    return false;
  }
  if (!future_result.get())
  {
    return false;
  }
  return true;
}

void PoseEstimationManager::add_camera_parameter(const std::string &name, const rclcpp::ParameterValue &value)
{
  camera_parameters_.emplace_back(rclcpp::Parameter(name, value));
}

void PoseEstimationManager::clear_camera_parameters()
{
  camera_parameters_.clear();
}