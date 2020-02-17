#pragma once
#include "chessboard_pose_estimator.hpp"
#include <xtensor/xarray.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pose_estimation_interface/srv/estimate_pose.hpp>

namespace pose_estimation
{
class PoseEstimation : public rclcpp_lifecycle::LifecycleNode
{
public:
	PoseEstimation(const rclcpp::NodeOptions &options);

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	on_configure(const rclcpp_lifecycle::State &state);
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	on_activate(const rclcpp_lifecycle::State &state);
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	on_deactivate(const rclcpp_lifecycle::State &state);
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	on_cleanup(const rclcpp_lifecycle::State &state);
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	on_shutdown(const rclcpp_lifecycle::State &state);

private:
	CPE::ChessboardPoseEstimator chessboard_pose_estimator;
	void publish_pose(std::vector<float> &pose_estimate);
	void estimate_pose(std::vector<float> &pose_estimate);
	rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>::SharedPtr object_pose_pub_;

	rclcpp::Service<pose_estimation_interface::srv::EstimatePose>::SharedPtr estimate_pose_service_;
	void estimate_pose_service_handler(
			const std::shared_ptr<rmw_request_id_t> request_header,
			const std::shared_ptr<pose_estimation_interface::srv::EstimatePose::Request> request,
			std::shared_ptr<pose_estimation_interface::srv::EstimatePose::Response> response);

	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
	// rclcpp_lifecycle::LifecycleNode::SharedPtr point_cloud_node_;
	void point_cloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
	


	//point cloud stuff
	sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_;
	void create_point_tensors(xt::xarray<float> &xyz, xt::xarray<int> &rgb);

	xt::xarray<float> xyz_;
	xt::xarray<int> rgb_;
	bool pose_estimation_success_;
	bool pnt_cld_recieved_;

};

} // namespace pose_estimation