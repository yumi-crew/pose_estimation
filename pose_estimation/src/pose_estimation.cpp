#include <pose_estimation.hpp>

using namespace std::placeholders;

namespace pose_estimation
{
PoseEstimation::PoseEstimation(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("pose_estimation", options),
																																		 pose_estimation_success_{false},
																																		 pnt_cld_recieved_{false}
{
	chessboard_pose_estimator = CPE::ChessboardPoseEstimator();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoseEstimation::on_configure(const rclcpp_lifecycle::State &state)
{
	//create services
	estimate_pose_service_ = create_service<pose_estimation_interface::srv::EstimatePose>(
			"estimate_pose", std::bind(&PoseEstimation::estimate_pose_service_handler, this, _1, _2, _3));

	object_pose_pub_ = create_publisher<geometry_msgs::msg::Pose>("object_pose", 10);
	RCLCPP_INFO_STREAM(get_logger(), "Configured.");
	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoseEstimation::on_activate(const rclcpp_lifecycle::State &state)
{

	object_pose_pub_->on_activate();

	point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"points", 10, std::bind(&PoseEstimation::point_cloud_sub_callback, this, _1));
	RCLCPP_INFO_STREAM(get_logger(), "Activated.");
	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoseEstimation::on_deactivate(const rclcpp_lifecycle::State &state)
{
	object_pose_pub_->on_deactivate();
	RCLCPP_INFO_STREAM(get_logger(), "Deactivated.");
	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoseEstimation::on_cleanup(const rclcpp_lifecycle::State &state)
{
	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoseEstimation::on_shutdown(const rclcpp_lifecycle::State &state)
{
	RCLCPP_INFO_STREAM(get_logger(), "Shutdown.");
	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void PoseEstimation::estimate_pose_service_handler(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<pose_estimation_interface::srv::EstimatePose::Request> request,
		std::shared_ptr<pose_estimation_interface::srv::EstimatePose::Response> response)
{
	std::vector<float> pose_estimate;
	pose_estimate.reserve(7);
	if(pnt_cld_recieved_){
		estimate_pose(pose_estimate);
	}
	if (pose_estimation_success_)
	{
		publish_pose(pose_estimate);
	}
	response->success = pose_estimation_success_;
}

void PoseEstimation::point_cloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg)
{
	point_cloud_ = point_cloud_msg;
	if (!pnt_cld_recieved_)
	{
		pnt_cld_recieved_ = true;
	}
}

void PoseEstimation::publish_pose(std::vector<float> &pose_estimate)
{
	//generate pose msg
	geometry_msgs::msg::Pose pose;
	pose.position.x = pose_estimate[0];
	pose.position.y = pose_estimate[1];
	pose.position.z = pose_estimate[2];
	pose.orientation.x = pose_estimate[3];
	pose.orientation.y = pose_estimate[4];
	pose.orientation.z = pose_estimate[5];
	pose.orientation.w = pose_estimate[6];
	// publish pose
	object_pose_pub_->publish(pose);
}

void PoseEstimation::estimate_pose(std::vector<float> &pose_estimate)
{
	create_point_tensors(xyz_, rgb_);
	// estimate pose
	chessboard_pose_estimator.set_point_cloud(xyz_, rgb_);
	pose_estimation_success_ = chessboard_pose_estimator.find_corners(8, 5);

	if (pose_estimation_success_)
	{
		chessboard_pose_estimator.extract_feature_pnt_cld();
		pose_estimate = chessboard_pose_estimator.estimate_pose();
	}
}

void PoseEstimation::create_point_tensors(xt::xarray<float> &xyz, xt::xarray<int> &rgb)
{
	auto width = static_cast<int>(point_cloud_->width);
	auto height = static_cast<int>(point_cloud_->height);

	xyz = xt::zeros<float>({height, width, 3});
	rgb = xt::zeros<int>({height, width, 3});

	sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_, "z");
	sensor_msgs::PointCloud2Iterator<int> iter_r(*point_cloud_, "r");
	sensor_msgs::PointCloud2Iterator<int> iter_g(*point_cloud_, "g");
	sensor_msgs::PointCloud2Iterator<int> iter_b(*point_cloud_, "b");

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
		{
			xyz(i, j, 0) = *iter_x;
			xyz(i, j, 1) = *iter_y;
			xyz(i, j, 2) = *iter_z;
			rgb(i, j, 0) = *iter_r;
			rgb(i, j, 1) = *iter_g;
			rgb(i, j, 2) = *iter_b;
		}
	}
}

} //namespace pose_estimation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pose_estimation::PoseEstimation)