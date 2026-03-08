#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Geometry>

static double height_, width_, platform_r_;

rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr polygen_pub_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cylinder_pub_;
std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  Eigen::Vector3d odom_p(msg->pose.pose.position.x,
                         msg->pose.pose.position.y,
                         msg->pose.pose.position.z);
  Eigen::Quaterniond odom_q(msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z);

  // 发布TF
  geometry_msgs::msg::TransformStamped transform;
  transform.header = msg->header;
  transform.child_frame_id = "target_platform";
  transform.transform.translation.x = msg->pose.pose.position.x;
  transform.transform.translation.y = msg->pose.pose.position.y;
  transform.transform.translation.z = msg->pose.pose.position.z;
  transform.transform.rotation = msg->pose.pose.orientation;
  broadcaster_->sendTransform(transform);

  // 发布平台矩形（CUBE）
  visualization_msgs::msg::Marker polygon;
  polygon.header = msg->header;
  polygon.type = visualization_msgs::msg::Marker::CUBE;
  polygon.pose = msg->pose.pose;
  polygon.scale.x = height_;
  polygon.scale.y = width_;
  polygon.scale.z = 0.02;
  polygon.color.a = 0.5;
  polygon.color.r = 0.0;
  polygon.color.g = 0.0;
  polygon.color.b = 1.0;
  polygen_pub_->publish(polygon);

  // 发布平台圆柱体（表示着陆区域）
  visualization_msgs::msg::Marker cylinder;
  cylinder.header = msg->header;
  cylinder.type = visualization_msgs::msg::Marker::CYLINDER;
  cylinder.pose = msg->pose.pose;
  cylinder.scale.x = platform_r_ * 2;
  cylinder.scale.y = platform_r_ * 2;
  cylinder.scale.z = 0.01;
  cylinder.color.a = 0.3;
  cylinder.color.r = 0.0;
  cylinder.color.g = 1.0;
  cylinder.color.b = 0.0;
  cylinder_pub_->publish(cylinder);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("odom_visualization_plate");

  // Declare parameters
  node->declare_parameter<double>("height", 1.2);
  node->declare_parameter<double>("width", 1.6);
  node->declare_parameter<double>("platform_r", 1.0);

  // Get parameters
  node->get_parameter("height", height_);
  node->get_parameter("width", width_);
  node->get_parameter("platform_r", platform_r_);

  // Create TF broadcaster
  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node);

  auto sub_odom = node->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 100, odom_callback);
  polygen_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("polygon", 100);
  cylinder_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("cylinder", 100);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
