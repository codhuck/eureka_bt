#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <future>

class Turn_inside : public BT::SyncActionNode, public rclcpp::Node
{
public:
    Turn_inside(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();

protected:
    BT::NodeStatus tick() override;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscribe_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_turn;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_future;

    void updateGoalPose(double turn_angle);

private:
    double pose_x_;
    double pose_y_;
    double orientationw;
    double orientationx;
    double orientationy;
    double orientationz;
};

