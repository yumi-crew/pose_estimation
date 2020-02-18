#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

class PoseListener : public rclcpp::Node
{
public:
  PoseListener(const std::string node_name);

  std::vector<float> get_pose();

private:
  void pose_estimation_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Pose>> pose_sub_;
  std::vector<float> pose_msg_;
};
