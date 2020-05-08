#pragma once
#include "chessboard_pose_estimator.hpp"
#include "opencv_surface_match.hpp"
#include "halcon_surface_match.hpp"

#include <iostream>
#include <unistd.h>
#include <xtensor/xarray.hpp>
#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pose_estimation_interface/srv/estimate_pose.hpp>
#include <pose_estimation_interface/srv/init_cv_surface_match.hpp>
#include <pose_estimation_interface/srv/init_halcon_surface_match.hpp>

#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

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
    /** 
    * Finds planes in the point cloud using a ransac scheme, removes points close to the plane.
    * @param cloud pointer to point cloud to remove planes from.
    * @param num_planes number of planes to be removed from the point cloud.
    * @return true if removal was successful.  
    **/
    bool remove_planes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int num_planes);

    /**
    * Generates and publishes a geometry_msgs::msg::PoseStamped message from a std::vector containing position x, y, z,
    * and orientation represented by a unit quaternion q1, q2, q3, q4, where q4 is the scalar part. The pose is published
    * to the topic /object_pose.
    * @param pose_estimate std::vector<float> {x, y, z, q1, q2, q3, q4}
    **/
    void publish_pose(std::vector<float> &pose_estimate);

    /**
    * Estimates the pose of a named object using the appropriate method set by initialization.
    * @param object name of object to find.
    * @param pose_estimate vector to store the result of pose estimation. 
    **/
    void estimate_pose(std::string object, std::vector<float> &pose_estimate);
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_pub_;

    /**
    * Handles calls from pose_estimation_interface::srv::EstimatePose services.
    * @param request EstimatePose.srv request, request->object contains the name (std::string) of the object to be found.
    * @param response EstimatePose.srv respose, response->success is true if call is successful.
    **/
    void estimate_pose_service_handler(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<pose_estimation_interface::srv::EstimatePose::Request> request,
        std::shared_ptr<pose_estimation_interface::srv::EstimatePose::Response> response);

    /**
    * Handles calls to pose_estimation_interface::srv::InitCvSurfaceMatch service.
    * Initializes opencv surface match functionality. Incompatible with InitHalconSurfaceMatch.
    * @param request InitCvSurfaceMatch.srv request, request->model_path_dir contains a string with the path to where the
    * models used for matching are stored. request->num_planes determines the number of planes to remove from the scene,
    * this is stored in PoseEstimation::num_planes_.
    * @param response InitCvSurfaceMatch.srv response, response->success is true if call is successful.
    **/
    void init_cv_surface_match_service_handler(const std::shared_ptr<rmw_request_id_t> request_header,
                                               const std::shared_ptr<pose_estimation_interface::srv::InitCvSurfaceMatch::Request> request,
                                               std::shared_ptr<pose_estimation_interface::srv::InitCvSurfaceMatch::Response> response);

    /**
    * Handles calls to pose_estimation_interface::srv::InitHalconSurfaceMatch service.
    * Initializes halcon surface match functionality. Incompatible with InitCvSurfaceMatch.
    * @param request InitHalconSurfaceMatch.srv request, request->model_path_dir contains a string with the path to where the
    * models used for matching are stored. request->num_planes determines the number of planes to remove from the scene,
    * this is stored in PoseEstimation::num_planes_.
    * @param response InitHalconSurfaceMatch.srv response, response->success is true if call is successful.
    **/
    void init_halcon_surface_match_service_handler(const std::shared_ptr<rmw_request_id_t> request_header,
                                                   const std::shared_ptr<pose_estimation_interface::srv::InitHalconSurfaceMatch::Request> request,
                                                   std::shared_ptr<pose_estimation_interface::srv::InitHalconSurfaceMatch::Response> response);

    /**
    * Callback for subscription to the topic /points published by zivid_ros. Stores the pointer to the published 
    * PointCloud2 msg in the member variable PoseEstimation::point_cloud_
    **/
    void point_cloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
      * Creates point arrays used by chessboard_pose_estimator.
      * @param xyz (NxMx3) xtensor array of the points in the scene.
      * @param rgb (NxMx3) xtensor array of the color intensity in the scene.
    **/  
    void create_point_tensors(xt::xarray<float> &xyz, xt::xarray<int> &rgb);

    /**
      * Converts the PointCloud2 msg into a format usable by HalconSurfaceMatch or OpenCvSurfaceMatch depending upon whether
      * PoseEstimation::use_halcon_match_ is true or false. For halcon surface match the scene is stored as a .ply file 
      * in a fixed location. For opencv surface match a cv::Mat of the points is returned.
      * @param num_planes number of planes to remove from the scene.
      * @returns cv::Mat representation of the scene.
    **/    
    cv::Mat create_surface_match_pc(int num_planes);

    rclcpp::Service<pose_estimation_interface::srv::EstimatePose>::SharedPtr estimate_pose_service_;
    rclcpp::Service<pose_estimation_interface::srv::InitCvSurfaceMatch>::SharedPtr init_cv_surface_match_service_;
    rclcpp::Service<pose_estimation_interface::srv::InitHalconSurfaceMatch>::SharedPtr init_halcon_surface_match_service_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_;

    ChessboardPoseEstimator chessboard_pose_estimator;
    OpenCVSurfaceMatch cv_surface_match;
    HalconSurfaceMatch halcon_surface_match;
    bool use_halcon_match_;
    int num_planes_;
    /*Path to the current scene, for halcon surface match.*/
    std::string path_to_scene_;
    xt::xarray<float> xyz_;
    xt::xarray<int> rgb_;
    bool pose_estimation_success_;
    bool pnt_cld_recieved_;
  };
} // namespace pose_estimation