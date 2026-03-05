#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <eigen3/Eigen/Geometry>

static double height_, width_, platform_r_;

rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr polygen_pub_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cylinder_pub_;

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  Eigen::Vector3d odom_p(msg->pose.pose.position.x,
                         msg->pose.pose.position.y,
                         msg->pose.pose.position.z);
  Eigen::Quaterniond odom_q(msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z);

  Eigen::MatrixXd R = odom_q.toRotationMatrix();
  double dx = height_ / 2;
  double dy = width_ / 2;

  visualization_msgs::msg::Marker polygon;
  polygon.header = msg->header;
  polygon.type = visualization_msgs::msg::Marker::CUBE;
  polygon.pose = msg->pose.pose;
  polygon.scale.x = height_ / 2;
  polygon.scale.y = width_ / 2;
  polygon.scale.z = 0.01;
  polygon.color.a = 0.5;
  polygon.color.r = 0.5;
  polygon.color.g = 0.5;
  polygon.color.b = 0.5;
  polygen_pub_->publish(polygon);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("odom_visualization_plate");

  node->declare_parameter("height", 1.2);
  node->declare_parameter("width", 1.6);
  node->declare_parameter("platform_r", 1.0);

  node->get_parameter("height", height_);
  node->get_parameter("width", width_);
  node->get_parameter("platform_r", platform_r_);

  auto sub_odom = node->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 100, odom_callback);
  polygen_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("polygon", 100);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
