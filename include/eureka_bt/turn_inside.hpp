#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <eureka_bt/bt_action_node.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <future>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class Turn_inside : public BT::SyncActionNode, public rclcpp::Node
{
public:
    Turn_inside(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;


private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscribe_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_turn;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_future;

    void updateGoalPose(double turn_angle);

};

