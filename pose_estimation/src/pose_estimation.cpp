#include <pose_estimation.hpp>

namespace pose_estimation
{
PoseEstimation::PoseEstimation(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("pose_estimation", options),
                                                                     pose_estimation_success_{false},
                                                                     pnt_cld_recieved_{false}
{
  chessboard_pose_estimator = ChessboardPoseEstimator();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoseEstimation::on_configure(const rclcpp_lifecycle::State &state)
{
  //create services
  estimate_pose_service_ = create_service<pose_estimation_interface::srv::EstimatePose>(
      "estimate_pose", std::bind(&PoseEstimation::estimate_pose_service_handler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  init_cv_surface_match_service_ = create_service<pose_estimation_interface::srv::InitCvSurfaceMatch>(
      "init_cv_surface_match", std::bind(&PoseEstimation::init_cv_surface_match_service_handler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  init_halcon_surface_match_service_ = create_service<pose_estimation_interface::srv::InitHalconSurfaceMatch>(
      "init_halcon_surface_match", std::bind(&PoseEstimation::init_halcon_surface_match_service_handler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  object_pose_pub_ = create_publisher<geometry_msgs::msg::Pose>("object_pose", 10);
  RCLCPP_INFO_STREAM(get_logger(), "Configured.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoseEstimation::on_activate(const rclcpp_lifecycle::State &state)
{

  object_pose_pub_->on_activate();

  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", 10, std::bind(&PoseEstimation::point_cloud_sub_callback, this, std::placeholders::_1));
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
  if (pnt_cld_recieved_)
  {
    estimate_pose(request->object, pose_estimate);
  }
  if (pose_estimation_success_)
  {
    publish_pose(pose_estimate);
  }
  response->success = pose_estimation_success_;
  pose_estimation_success_ = false;
}

void PoseEstimation::init_cv_surface_match_service_handler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<pose_estimation_interface::srv::InitCvSurfaceMatch::Request> request,
    std::shared_ptr<pose_estimation_interface::srv::InitCvSurfaceMatch::Response> response)
{
  cv_surface_match.load_models_from_dir(request->model_dir_path);
  cv_surface_match.train_models();
  response->success = true;
  use_halcon_match = false;
}

void PoseEstimation::init_halcon_surface_match_service_handler(
     const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<pose_estimation_interface::srv::InitHalconSurfaceMatch::Request> request,
    std::shared_ptr<pose_estimation_interface::srv::InitHalconSurfaceMatch::Response> response)
{
  halcon_surface_match.load_models(request->model_dir_path);
  response->success = true;
  use_halcon_match = true;
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

void PoseEstimation::estimate_pose(std::string object, std::vector<float> &pose_estimate)
{
  if (object.compare("chessboard") == 0)
  {
    create_point_tensors(xyz_, rgb_);
    // estimate pose
    chessboard_pose_estimator.set_point_cloud(xyz_, rgb_);
    pose_estimation_success_ = chessboard_pose_estimator.find_corners(19, 12); // 8, 5

    if (pose_estimation_success_)
    {
      chessboard_pose_estimator.extract_feature_pnt_cld();
      pose_estimate = chessboard_pose_estimator.estimate_pose();
    }
  }
  else if(use_halcon_match)
  {
    create_cv_pc();
    halcon_surface_match.update_current_scene();
    pose_estimate = halcon_surface_match.find_object_in_scene(object);
    pose_estimation_success_ = true;
  }
  else
  {
    cv::Mat pc = create_cv_pc();
    pose_estimate = cv_surface_match.find_object_in_scene(object, pc);
    pose_estimation_success_ = true;
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

cv::Mat PoseEstimation::create_cv_pc()
{
  // creates Nx3 cv mat for point cloud representation. Removes ground plane
  auto width = static_cast<int>(point_cloud_->width);
  auto height = static_cast<int>(point_cloud_->height);
  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_, "z");
  sensor_msgs::PointCloud2Iterator<int> iter_r(*point_cloud_, "r");
  sensor_msgs::PointCloud2Iterator<int> iter_g(*point_cloud_, "g");
  sensor_msgs::PointCloud2Iterator<int> iter_b(*point_cloud_, "b");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_idx(new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  cloud->width = width * height;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  coefficients->values.resize(4); // ax+by+cz+d=0

  for (int i = 0; i < width * height; ++i, ++iter_x, ++iter_y, ++iter_z) //, ++iter_r, ++iter_g, ++iter_b)
  {
    cloud->points[i].x = *iter_x;
    cloud->points[i].y = *iter_y;
    cloud->points[i].z = *iter_z;
    // cloud->points[i].r = *iter_r;
    // cloud->points[i].g = *iter_g;
    // cloud->points[i].b = *iter_b;
  }
  cloud->is_dense = false;
  std::vector<int> idx;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, idx);

  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setOptimizeCoefficients(true);
  seg.setDistanceThreshold(0.0001);
  seg.setMaxIterations(1000);
  seg.setInputCloud(cloud);
  seg.segment(*inliers_idx, *coefficients);

  if (inliers_idx->indices.size() == 0)
  {
    PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
  }

  std::cout << "plane coefficients " << coefficients->values[0] << " "
            << coefficients->values[1] << " "
            << coefficients->values[2] << " "
            << coefficients->values[3] << std::endl;
  //use plane coefficients to calculate distance to plane for each point, filter out near points
  pcl::PointIndices::Ptr pts_close_to_plane_idx(new pcl::PointIndices);
  for (uint i = 0; i < cloud->points.size(); ++i)
  {
    float d = (cloud->points[i].x * coefficients->values[0] +
               cloud->points[i].y * coefficients->values[1] +
               cloud->points[i].z * coefficients->values[2] +
               coefficients->values[3]) /
              std::sqrt(cloud->points[i].x * cloud->points[i].x +
                        cloud->points[i].y * cloud->points[i].y +
                        cloud->points[i].z * cloud->points[i].z);
    if (std::abs(d) <= 0.01)
    {
      pts_close_to_plane_idx->indices.push_back(i);
    }
  }
  //extract outliers of ground plane
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(pts_close_to_plane_idx);
  extract.setNegative(false);
  extract.filter(*inliers);
  extract.setNegative(true);
  extract.filter(*outliers); //outliers now contains the points not on the ground plane

  pcl::io::savePLYFileASCII("current_scene.ply", *outliers);

  // Visualization
  // pcl::visualization::PCLVisualizer viewer("PCL visualizer");

  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_inliers_handler(cloud, 255, 20, 20); // Plane in RED
  // viewer.addPointCloud(inliers, cloud_inliers_handler, "cloud inliers");

  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_outliers_handler(cloud, 200, 200, 200);
  // viewer.addPointCloud(outliers, cloud_outliers_handler, "cloud outliers");

  // while (!viewer.wasStopped())
  // {
  //   viewer.spinOnce();
  // }
  // viewer.close();
  cv::Mat pc = cv::Mat::zeros(outliers->width * outliers->height, 3, CV_32F);
  std::cout << pc.size;
  for (uint i = 0; i < outliers->width * outliers->height; ++i)
  {
    pc.at<cv::Vec3f>(i)[0] = outliers->points[i].x;
    pc.at<cv::Vec3f>(i)[1] = outliers->points[i].y;
    pc.at<cv::Vec3f>(i)[2] = outliers->points[i].z;
  }
  return pc;
}

} //namespace pose_estimation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pose_estimation::PoseEstimation)