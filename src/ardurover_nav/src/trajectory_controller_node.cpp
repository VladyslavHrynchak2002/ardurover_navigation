#include "ardurover_nav/path_io.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace ardurover_nav {

class TrajectoryControllerNode : public rclcpp::Node {
  public:
    TrajectoryControllerNode() : Node("trajectory_controller_node") {
        pathFile_ = declare_parameter("path_file", std::string("paths/example.path"));
        vMax_ = declare_parameter("v_max", 1.2);
        wMax_ = declare_parameter("w_max", 1.0);
        const double rateHz = declare_parameter("control_rate_hz", 20.0);

        path_ = load_path(pathFile_);
        if (path_.empty()) {
            throw std::runtime_error("Path file is empty: " + pathFile_);
        }

        cmdPub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/mavros/setpoint_velocity/cmd_vel_unstamped",
            10
        );
        odomSub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth/odom",
            10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { latestOdom_ = std::move(msg); }
        );
        arming_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        setMode_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        paramClient_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/mavros/setpoint_velocity");

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rateHz),
            std::bind(&TrajectoryControllerNode::OnTimer, this)
        );

        RCLCPP_INFO_STREAM(
            get_logger(),
            "Loaded " << path_.size() << " waypoints from " << pathFile_
        );
    }

  private:
    enum class SetupState { WaitServices, SetFrame, Prime, SetMode, Arm, Ready };

    // Implement path following here.
    //
    // Input:  latest Gazebo pose/twist in latestOdom_, reference path in path_
    //         (x, y in metres, yaw in radians, world ENU, no timestamps).
    // Output: body-frame Twist on /mavros/setpoint_velocity/cmd_vel_unstamped
    //         linear.x  = forward speed (m/s)
    //         angular.z = yaw rate (rad/s)
    // Do not upload missions or publish position setpoints.
    geometry_msgs::msg::Twist ComputeCommand(const nav_msgs::msg::Odometry& /*state*/) {
        geometry_msgs::msg::Twist command;
        return command;
    }

    void OnTimer() {
        if (setupState_ != SetupState::Ready) {
            AdvanceSetup();
            geometry_msgs::msg::Twist prime;
            prime.linear.x = 0.001;
            cmdPub_->publish(prime);
            return;
        }

        if (!latestOdom_) {
            return;
        }

        geometry_msgs::msg::Twist command = ComputeCommand(*latestOdom_);
        command.linear.x = std::clamp(command.linear.x, -vMax_, vMax_);
        command.angular.z = std::clamp(command.angular.z, -wMax_, wMax_);
        cmdPub_->publish(command);
    }

    void AdvanceSetup() {
        switch (setupState_) {
            case SetupState::WaitServices:
                if (arming_->service_is_ready() && setMode_->service_is_ready()) {
                    setupState_ = SetupState::SetFrame;
                }
                return;
            case SetupState::SetFrame:
                if (!paramClient_->service_is_ready()) {
                    return;
                }
                paramClient_->set_parameters({rclcpp::Parameter("mav_frame", "BODY_NED")});
                RCLCPP_INFO(get_logger(), "Setpoint frame set to BODY_NED");
                setupState_ = SetupState::Prime;
                primeTicks_ = 0;
                return;
            case SetupState::Prime:
                ++primeTicks_;
                if (primeTicks_ >= 20) {
                    setupState_ = SetupState::SetMode;
                }
                return;
            case SetupState::SetMode:
                if (!modeFuture_.valid()) {
                    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                    req->custom_mode = "GUIDED";
                    modeFuture_ = setMode_->async_send_request(req).future.share();
                    RCLCPP_INFO(get_logger(), "Requesting GUIDED");
                } else if (modeFuture_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                    setupState_ = SetupState::Arm;
                }
                return;
            case SetupState::Arm:
                if (!armFuture_.valid()) {
                    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                    req->value = true;
                    armFuture_ = arming_->async_send_request(req).future.share();
                    RCLCPP_INFO(get_logger(), "Requesting arm");
                } else if (armFuture_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                    setupState_ = SetupState::Ready;
                    RCLCPP_INFO(get_logger(), "Controller running");
                }
                return;
            case SetupState::Ready:
                return;
        }
    }

    std::string pathFile_;
    double vMax_{1.2};
    double wMax_{1.0};
    std::vector<Waypoint> path_;
    nav_msgs::msg::Odometry::ConstSharedPtr latestOdom_;

    SetupState setupState_{SetupState::WaitServices};
    int primeTicks_{0};
    std::shared_future<mavros_msgs::srv::SetMode::Response::SharedPtr> modeFuture_;
    std::shared_future<mavros_msgs::srv::CommandBool::Response::SharedPtr> armFuture_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr setMode_;
    rclcpp::AsyncParametersClient::SharedPtr paramClient_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ardurover_nav

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ardurover_nav::TrajectoryControllerNode>());
    rclcpp::shutdown();
    return 0;
}
