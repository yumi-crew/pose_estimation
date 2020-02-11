#include <pose_estimation.hpp>

using namespace std::placeholders;

namespace pose_estimation
{
PoseEstimation::PoseEstimation(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("pose_estimation", options)
{
    chessboard_pose_estimator = CPE::ChessboardPoseEstimator();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoseEstimation::on_configure(const rclcpp_lifecycle::State &state)
{
    estimate_pose_service_ = create_service<pose_estimation_interface::srv::EstimatePose>(
        "estimate_pose", std::bind(&PoseEstimation::estimate_pose_service_handler, this, _1, _2, _3));

    point_cloud_node_ = rclcpp::Node::make_shared("point_cloud_node");
    auto point_cloud_sub = point_cloud_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points", 10, std::bind(&PoseEstimation::point_cloud_sub_callback, this, _1));
    object_pose_pub_ = create_publisher<geometry_msgs::msg::Pose>("object_pose", 10);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoseEstimation::on_activate(const rclcpp_lifecycle::State &state)
{
    object_pose_pub_->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoseEstimation::on_deactivate(const rclcpp_lifecycle::State &state)
{
    object_pose_pub_->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_cleanup(const rclcpp_lifecycle::State &state)
{
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_shutdown(const rclcpp_lifecycle::State &state)
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
}

void PoseEstimation::publish_pose()
{
    //estimate pose
    // chessboard_pose_estimator.set_point_cloud(img, xyz);
    // std::vector pose_estimate = chessboard_pose_estimator.estimate_pose();
    // //generate pose msg
    // geometry_msgs::msg::Pose pose;
    // pose.position.x = pose_estimate[0];
    // pose.position.y = pose_estimate[1];
    // pose.position.z = pose_estimate[2];
    // pose.orientation.x = pose_estimate[3];
    // pose.orientation.y = pose_estimate[4];
    // pose.orientation.z = pose_estimate[5];
    // pose.orientation.w = pose_estimate[6];
    // //publish pose
    // object_pose_pub_->publish(pose);
}

} //namespace pose_estimation