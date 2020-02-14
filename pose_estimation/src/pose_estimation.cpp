#include <pose_estimation.hpp>

using namespace std::placeholders;

namespace pose_estimation
{
PoseEstimation::PoseEstimation(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("pose_estimation", options)
{
	RCLCPP_INFO_STREAM(this->get_logger(), "hei_og_hopp");
	chessboard_pose_estimator = CPE::ChessboardPoseEstimator();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoseEstimation::on_configure(const rclcpp_lifecycle::State &state)
{
	//create services
	estimate_pose_service_ = create_service<pose_estimation_interface::srv::EstimatePose>(
		"estimate_pose", std::bind(&PoseEstimation::estimate_pose_service_handler, this, _1, _2, _3));

	RCLCPP_INFO_STREAM(this->get_logger(), "hei_og_hopp");
	//configure nodes for subs and pubs
	// point_cloud_node_("point_cloud_node");
	// point_cloud_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
	// 		"points", 10, std::bind(&PoseEstimation::point_cloud_sub_callback, this, _1));
	RCLCPP_INFO_STREAM(this->get_logger(), "hei_og_hopp");
	// point_cloud_node_ = rclcpp::Node::make_shared("point_cloud_node");
	// auto point_cloud_sub_ = point_cloud_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
	// 		"points", 10, std::bind(&PoseEstimation::point_cloud_sub_callback, this, _1));

	object_pose_pub_ = create_publisher<geometry_msgs::msg::Pose>("object_pose", 10);

	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoseEstimation::on_activate(const rclcpp_lifecycle::State &state)
{

	object_pose_pub_->on_activate();

	point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		"points", 10, std::bind(&PoseEstimation::point_cloud_sub_callback, this, _1));

	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoseEstimation::on_deactivate(const rclcpp_lifecycle::State &state)
{
	// point_cloud_node_->on_deactivate(state);
	object_pose_pub_->on_deactivate();
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
	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void PoseEstimation::estimate_pose_service_handler(
	const std::shared_ptr<rmw_request_id_t> request_header,
	const std::shared_ptr<pose_estimation_interface::srv::EstimatePose::Request> request,
	std::shared_ptr<pose_estimation_interface::srv::EstimatePose::Response> response)
{
	publish_pose();
}

void PoseEstimation::point_cloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg)
{
	point_cloud_ = point_cloud_msg;
	RCLCPP_INFO_STREAM(this->get_logger(), "point_cloud_sub_callback");
}

void PoseEstimation::publish_pose()
{
	create_point_tensors(xyz_, rgb_);
	// estimate pose
	chessboard_pose_estimator.set_point_cloud(xyz_, rgb_);
	chessboard_pose_estimator.find_corners();
	chessboard_pose_estimator.extract_feature_pnt_cld();
	std::vector<float> pose_estimate = chessboard_pose_estimator.estimate_pose();
	
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

void PoseEstimation::create_point_tensors(xt::xarray<float> &xyz, xt::xarray<int> &rgb)
{
	RCLCPP_INFO_STREAM(this->get_logger(), "create tensors");

	auto width = static_cast<int>(point_cloud_->width);
	auto height = static_cast<int>(point_cloud_->height);

	xyz = xt::zeros<float>({height, width, 3});
	rgb = xt::zeros<int>({height, width, 3});

	// xyz.resize({point_cloud_->height, point_cloud_->width, 3});
	// rgb.resize({point_cloud_->height, point_cloud_->width, 3});

	RCLCPP_INFO_STREAM(this->get_logger(), "reshape ok");

	sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_, "z");
	sensor_msgs::PointCloud2Iterator<int> iter_r(*point_cloud_, "r");
	sensor_msgs::PointCloud2Iterator<int> iter_g(*point_cloud_, "g");
	sensor_msgs::PointCloud2Iterator<int> iter_b(*point_cloud_, "b");
	RCLCPP_INFO_STREAM(this->get_logger(), "created iterators");

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
	RCLCPP_INFO_STREAM(this->get_logger(), "point tensor ok");
}

} //namespace pose_estimation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pose_estimation::PoseEstimation)