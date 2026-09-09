#include "ardurover_nav/ardurover_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

namespace ardurover_nav {

ArduroverController::ArduroverController(rclcpp::Node &node, std::vector<Waypoint> path)
    : node_(node), path_(std::move(path)) {
    arming_ = node_.create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    setMode_ = node_.create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    paramClient_ = std::make_shared<rclcpp::AsyncParametersClient>(&node_, "/mavros/setpoint_velocity");
    cmdVelPub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", rclcpp::SensorDataQoS()
    );

    lookahead_ = node_.declare_parameter("lookahead_m", lookahead_);
    cruiseSpeed_ = node_.declare_parameter("cruise_speed_mps", cruiseSpeed_);
    maxAngularSpeed_ = node_.declare_parameter("max_angular_speed_rps", maxAngularSpeed_);
    goalRadius_ = node_.declare_parameter("goal_radius_m", goalRadius_);
}

bool ArduroverController::SetupArdurover() {
    switch (setupState_) {
        case SetupState::WaitServices:
            if (arming_->service_is_ready() && setMode_->service_is_ready()) {
                setupState_ = SetupState::SetFrame;
            }
            return false;
        case SetupState::SetFrame:
            if (!paramClient_->service_is_ready()) {
                return false;
            }
            paramClient_->set_parameters({rclcpp::Parameter("mav_frame", "BODY_NED")});
            setupState_ = SetupState::Prime;
            return false;
        case SetupState::Prime:
            if (++primeTicks_ >= 20) {
                setupState_ = SetupState::SetMode;
            }
            return false;
        case SetupState::SetMode:
            if (!modeFuture_.valid()) {
                auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                req->custom_mode = "GUIDED";
                modeFuture_ = setMode_->async_send_request(req).future.share();
            } else if (modeFuture_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                setupState_ = SetupState::Arm;
            }
            return false;
        case SetupState::Arm:
            if (!armFuture_.valid()) {
                auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                req->value = true;
                armFuture_ = arming_->async_send_request(req).future.share();
                return false;
            }
            if (armFuture_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
                return false;
            }
            {
                const auto resp = armFuture_.get();
                if (resp && resp->success) {
                    setupState_ = SetupState::Ready;
                    RCLCPP_INFO(node_.get_logger(), "Controller running");
                    return true;
                }
                armFuture_ = {};
            }
            return false;
        case SetupState::Ready:
            return true;
    }
    return false;
}

size_t ArduroverController::ClosestWaypointIndex(double x, double y, size_t hint) const {
    const size_t searchWindow = 40;
    const size_t end = std::min(path_.size(), hint + searchWindow);

    size_t best = hint;
    double bestDist2 = std::numeric_limits<double>::infinity();
    for (size_t i = hint; i < end; ++i) {
        const double dx = path_[i].x - x;
        const double dy = path_[i].y - y;
        const double dist2 = dx * dx + dy * dy;
        if (dist2 < bestDist2) {
            bestDist2 = dist2;
            best = i;
        }
    }
    return best;
}

Waypoint ArduroverController::LookaheadPoint(size_t fromIdx, double x, double y) const {
    if (fromIdx >= path_.size() - 1) {
        return path_.back();
    }

    double traveled = std::hypot(path_[fromIdx].x - x, path_[fromIdx].y - y);
    if (traveled >= lookahead_) {
        return path_[fromIdx];
    }

    for (size_t i = fromIdx; i + 1 < path_.size(); ++i) {
        const double segLen = std::hypot(path_[i + 1].x - path_[i].x, path_[i + 1].y - path_[i].y);
        if (traveled + segLen >= lookahead_) {
            const double t = segLen < 1e-9 ? 1.0 : (lookahead_ - traveled) / segLen;
            Waypoint target;
            target.x = path_[i].x + t * (path_[i + 1].x - path_[i].x);
            target.y = path_[i].y + t * (path_[i + 1].y - path_[i].y);
            return target;
        }
        traveled += segLen;
    }
    return path_.back();
}

void ArduroverController::Control(const nav_msgs::msg::Odometry &odometry) {
    const double x = odometry.pose.pose.position.x;
    const double y = odometry.pose.pose.position.y;
    const double yaw = yaw_from_quat(odometry.pose.pose.orientation);

    closestIdx_ = ClosestWaypointIndex(x, y, closestIdx_);

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_.get_clock()->now();
    cmd.header.frame_id = "base_link";

    const double distToGoal = std::hypot(path_.back().x - x, path_.back().y - y);

    if (closestIdx_ >= path_.size() - 1 && distToGoal <= goalRadius_) {
        cmdVelPub_->publish(cmd);
        return;
    }

    const Waypoint target = LookaheadPoint(closestIdx_, x, y);

    const double dx = target.x - x;
    const double dy = target.y - y;
    double alpha = std::atan2(dy, dx) - yaw;
    alpha = std::atan2(std::sin(alpha), std::cos(alpha));  // wrap to [-pi, pi]
    const double Ld = std::max(std::hypot(dx, dy), 1e-3);
    const double curvature = 2.0 * std::sin(alpha) / Ld;

    const double turnSlowdown = 1.0 / (1.0 + 2.0 * std::abs(curvature));
    const double speed = std::clamp(cruiseSpeed_ * turnSlowdown, 0.3, cruiseSpeed_);

    cmd.twist.linear.x = speed;
    cmd.twist.angular.z = std::clamp(curvature * speed, -maxAngularSpeed_, maxAngularSpeed_);

    cmdVelPub_->publish(cmd);
}

}  // namespace ardurover_nav
