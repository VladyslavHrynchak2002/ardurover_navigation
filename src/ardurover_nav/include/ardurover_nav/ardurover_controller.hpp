#pragma once

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "ardurover_nav/path_io.hpp"

namespace ardurover_nav {

class ArduroverController {
  public:
    ArduroverController(rclcpp::Node& node, std::vector<Waypoint> path);

    bool SetupArdurover();
    void Control(const nav_msgs::msg::Odometry& odom);

  private:
    enum class SetupState { WaitServices, SetFrame, Prime, SetMode, Arm, Ready };

    size_t ClosestWaypointIndex(double x, double y, size_t hint) const;
    Waypoint LookaheadPoint(size_t fromIdx, double x, double y) const;

    rclcpp::Node& node_;
    std::vector<Waypoint> path_;
    SetupState setupState_{SetupState::WaitServices};
    int primeTicks_{0};
    std::shared_future<mavros_msgs::srv::SetMode::Response::SharedPtr> modeFuture_;
    std::shared_future<mavros_msgs::srv::CommandBool::Response::SharedPtr> armFuture_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr setMode_;
    rclcpp::AsyncParametersClient::SharedPtr paramClient_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmdVelPub_;

    // Pure pursuit tuning.
    double lookahead_{1.5};       // m
    double cruiseSpeed_{1.0};     // m/s, nominal forward speed
    double maxAngularSpeed_{1.5}; // rad/s
    double goalRadius_{0.5};      // m, distance to final waypoint that counts as "arrived"

    size_t closestIdx_{0};
};

}  // namespace ardurover_nav
