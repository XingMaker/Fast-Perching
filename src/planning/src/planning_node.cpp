#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <traj_opt/traj_opt.h>

#include <Eigen/Core>
#include <atomic>
#include <thread>
#include <vis_utils/vis_utils.hpp>

namespace planning {

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

class PlanningNode : public rclcpp::Node {
 private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr triger_sub_;
  rclcpp::TimerBase::SharedPtr plan_timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  std::shared_ptr<vis_utils::VisUtils> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;

  // NOTE planning or fake target
  bool target_ = false;
  Eigen::Vector3d goal_;

  // NOTE just for debug
  bool debug_ = false;
  bool once_ = false;
  bool debug_replan_ = false;

  double tracking_dur_, tracking_dist_, tolerance_d_;
  Eigen::Vector3d perching_p_, perching_v_, perching_axis_;
  double perching_theta_;

  Trajectory traj_poly_;
  rclcpp::Time replan_stamp_;
  int traj_id_ = 0;
  bool wait_hover_ = true;
  bool force_hover_ = true;

  int plan_hz_ = 10;

  std::atomic_bool triger_received_{false};

  void triger_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msgPtr) {
    goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 1.0;
    triger_received_ = true;
  }

  void debug_timer_callback() {
    if (!triger_received_) {
      return;
    }
    // Wait for initialization to complete
    if (!trajOptPtr_ || !visPtr_) {
      RCLCPP_WARN(this->get_logger(), "Waiting for initialization...");
      return;
    }
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 4);
    bool generate_new_traj_success = false;
    Trajectory traj;
    Eigen::Vector3d target_p, target_v;
    Eigen::Quaterniond target_q;
    Eigen::Quaterniond land_q(1, 0, 0, 0);

    iniState.setZero();
    iniState.col(0).x() = 0.0;
    iniState.col(0).y() = 0.0;
    iniState.col(0).z() = 2.0;
    iniState.col(1) = perching_v_;
    target_p = perching_p_;
    target_v = perching_v_;
    target_q.x() = 0.0;
    target_q.y() = 0.0;
    target_q.z() = 0.0;
    target_q.w() = 1.0;

    Eigen::Vector3d axis = perching_axis_.normalized();
    double theta = perching_theta_ * 0.5;
    land_q.w() = cos(theta);
    land_q.x() = axis.x() * sin(theta);
    land_q.y() = axis.y() * sin(theta);
    land_q.z() = axis.z() * sin(theta);
    land_q = target_q * land_q;

    std::cout << "iniState: \n"
              << iniState << std::endl;
    std::cout << "target_p: " << target_p.transpose() << std::endl;
    std::cout << "target_v: " << target_v.transpose() << std::endl;
    std::cout << "land_q: "
              << land_q.w() << ","
              << land_q.x() << ","
              << land_q.y() << ","
              << land_q.z() << "," << std::endl;

    generate_new_traj_success = trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj);
    if (generate_new_traj_success) {
      visPtr_->visualize_traj(traj, "traj");

      Eigen::Vector3d tail_pos = traj.getPos(traj.getTotalDuration());
      Eigen::Vector3d tail_vel = traj.getVel(traj.getTotalDuration());
      visPtr_->visualize_arrow(tail_pos, tail_pos + 0.5 * tail_vel, "tail_vel");
    }
    if (!generate_new_traj_success) {
      triger_received_ = false;
      return;
      // assert(false);
    }

    // NOTE run vis
    // hopf fiberation
    auto v2q = [](const Eigen::Vector3d& v, Eigen::Quaterniond& q) -> bool {
      double a = v.x();
      double b = v.y();
      double c = v.z();
      if (c == -1) {
        return false;
      }
      double d = 1.0 / sqrt(2.0 * (1 + c));
      q.w() = (1 + c) * d;
      q.x() = -b * d;
      q.y() = a * d;
      q.z() = 0;
      return true;
    };

    auto f_DN = [](const Eigen::Vector3d& x) {
      double x_norm_2 = x.squaredNorm();
      return (Eigen::MatrixXd::Identity(3, 3) - x * x.transpose() / x_norm_2) / sqrt(x_norm_2);
    };
    // auto f_D2N = [](const Eigen::Vector3d& x, const Eigen::Vector3d& y) {
    //   double x_norm_2 = x.squaredNorm();
    //   double x_norm_3 = x_norm_2 * x.norm();
    //   Eigen::MatrixXd A = (3 * x * x.transpose() / x_norm_2 - Eigen::MatrixXd::Identity(3, 3));
    //   return (A * y * x.transpose() - x * y.transpose() - x.dot(y) * Eigen::MatrixXd::Identity(3, 3)) / x_norm_3;
    // };

    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "world";
    double dt = 0.001;
    Eigen::Quaterniond q_last;
    double max_omega = 0;
    for (double t = 0; t <= traj.getTotalDuration(); t += dt) {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
      // drone
      Eigen::Vector3d p = traj.getPos(t);
      Eigen::Vector3d a = traj.getAcc(t);
      Eigen::Vector3d j = traj.getJer(t);
      Eigen::Vector3d g(0, 0, -9.8);
      Eigen::Vector3d thrust = a - g;

      // std::cout << p.x() << " , " << p.z() << " , ";

      Eigen::Vector3d zb = thrust.normalized();
      {
        // double a = zb.x();
        // double b = zb.y();
        // double c = zb.z();
        Eigen::Vector3d zb_dot = f_DN(thrust) * j;
        double omega12 = zb_dot.norm();
        // if (omega12 > 3.1) {
        //   std::cout << "omega: " << omega12 << "rad/s  t: " << t << std::endl;
        // }
        if (omega12 > max_omega) {
          max_omega = omega12;
        }
        // double a_dot = zb_dot.x();
        // double b_dot = zb_dot.y();
        // double omega3 = (b * a_dot - a * b_dot) / (1 + c);
        // std::cout << "jer: " << j.transpose() << std::endl;
        // std::cout << "omega12: " << zb_dot.norm() << std::endl;
        // std::cout << "omega3: " << omega3 << std::endl;
        // std::cout << thrust.x() << " , " << thrust.z() << " , ";
        // double omega2 = zb_dot.x() - zb.x() * zb_dot.z() / (zb.z() + 1);
        // std::cout << omega2 << std::endl;
        // std::cout << zb_dot.norm() << std::endl;
      }

      Eigen::Quaterniond q;
      bool no_singlarity = v2q(zb, q);
      Eigen::MatrixXd R_dot = (q.toRotationMatrix() - q_last.toRotationMatrix()) / dt;
      Eigen::MatrixXd omega_M = q.toRotationMatrix().transpose() * R_dot;
      // std::cout << "omega_M: \n" << omega_M << std::endl;
      Eigen::Vector3d omega_real;
      omega_real.x() = -omega_M(1, 2);
      omega_real.y() = omega_M(0, 2);
      omega_real.z() = -omega_M(0, 1);
      // std::cout << "omega_real: " << omega_real.transpose() << std::endl;
      q_last = q;
      if (no_singlarity) {
        msg.pose.pose.position.x = p.x();
        msg.pose.pose.position.y = p.y();
        msg.pose.pose.position.z = p.z();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.header.stamp = this->now();
        visPtr_->visualize_traj(traj, "traj");
        visPtr_->pub_msg(msg, "odom");
      }
      // target
      // Eigen::Vector3d fake_target_v = target_v * (1.0 + 0.5 * sin(1e6 * t));
      // target_p = target_p + fake_target_v * dt;
      // target_v *= 1.0001;
      target_p = target_p + target_v * dt;
      msg.pose.pose.position.x = target_p.x();
      msg.pose.pose.position.y = target_p.y();
      msg.pose.pose.position.z = target_p.z();
      msg.pose.pose.orientation.w = land_q.w();
      msg.pose.pose.orientation.x = land_q.x();
      msg.pose.pose.orientation.y = land_q.y();
      msg.pose.pose.orientation.z = land_q.z();
      msg.header.stamp = this->now();
      visPtr_->pub_msg(msg, "target_odom");
      if (trajOptPtr_->check_collilsion(p, a, target_p)) {
        std::cout << "collide!  t: " << t << std::endl;
      }
      // TODO replan
      if (debug_replan_ && t > 1.0 / plan_hz_ && traj.getTotalDuration() > 0.5) {
        // rclcpp::sleep_for(std::chrono::seconds(3));

        iniState.col(0) = traj.getPos(t);
        iniState.col(1) = traj.getVel(t);
        iniState.col(2) = traj.getAcc(t);
        iniState.col(3) = traj.getJer(t);
        std::cout << "iniState: \n"
                  << iniState << std::endl;
        trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj, t);
        visPtr_->visualize_traj(traj, "traj");
        t = 0;
        std::cout << "max omega: " << max_omega << std::endl;
      }
    }
    std::cout << "tailV: " << traj.getVel(traj.getTotalDuration()).transpose() << std::endl;
    std::cout << "max thrust: " << traj.getMaxThrust() << std::endl;
    std::cout << "max omega: " << max_omega << std::endl;

    triger_received_ = false;
  }

 public:
  PlanningNode(const rclcpp::NodeOptions &options) : Node("planning_node", options) {
    // Declare parameters
    this->declare_parameter("replan", false);
    this->declare_parameter("perching_px", 0.5);
    this->declare_parameter("perching_py", 0.0);
    this->declare_parameter("perching_pz", 2.0);
    this->declare_parameter("perching_vx", 2.0);
    this->declare_parameter("perching_vy", 0.0);
    this->declare_parameter("perching_vz", 0.0);
    this->declare_parameter("perching_axis_x", 0.0);
    this->declare_parameter("perching_axis_y", 1.0);
    this->declare_parameter("perching_axis_z", 0.0);
    this->declare_parameter("perching_theta", -1.5708);
    this->declare_parameter("plan_hz", 10);

    // Get parameters
    this->get_parameter("replan", debug_replan_);
    this->get_parameter("perching_px", perching_p_.x());
    this->get_parameter("perching_py", perching_p_.y());
    this->get_parameter("perching_pz", perching_p_.z());
    this->get_parameter("perching_vx", perching_v_.x());
    this->get_parameter("perching_vy", perching_v_.y());
    this->get_parameter("perching_vz", perching_v_.z());
    this->get_parameter("perching_axis_x", perching_axis_.x());
    this->get_parameter("perching_axis_y", perching_axis_.y());
    this->get_parameter("perching_axis_z", perching_axis_.z());
    this->get_parameter("perching_theta", perching_theta_);
    this->get_parameter("plan_hz", plan_hz_);

    // Delay initialization to allow shared_from_this to work
    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
          init_timer_->cancel();
          visPtr_ = std::make_shared<vis_utils::VisUtils>(this->shared_from_this());
          trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(this->shared_from_this());
          RCLCPP_WARN(this->get_logger(), "Planning node initialized!");
        });

    plan_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / plan_hz_),
        std::bind(&PlanningNode::debug_timer_callback, this));

    triger_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "triger", 10,
        std::bind(&PlanningNode::triger_callback, this, std::placeholders::_1));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planning::PlanningNode)

// Entry point for standalone executable
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<planning::PlanningNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
