#pragma once
#include "chessboard_pose_estimator.hpp"

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
    void publish_pose();
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>::SharedPtr object_pose_pub_;

    rclcpp::Service<pose_estimation_interface::srv::EstimatePose>::SharedPtr estimate_pose_service_;

    void estimate_pose_service_handler(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<pose_estimation_interface::srv::EstimatePose::Request> request,
        std::shared_ptr<pose_estimation_interface::srv::EstimatePose::Response> response);

    void point_cloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    rclcpp::Node::SharedPtr point_cloud_node_;

    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_;

    //use for iterating over the point cloud, local in func later...
    // sensor_msgs::PointCloud2ConstIterator<float*> point_cloud2_iter_xyz;
    // sensor_msgs::PointCloud2ConstIterator<uint8_t*> point_clud2_iter_rgb;
};

} //namespace pose_estimator