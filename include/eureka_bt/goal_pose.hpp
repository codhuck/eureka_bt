#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <eureka_bt/bt_action_node.hpp>


class Goalpose : public BT::SyncActionNode, public rclcpp::Node {
public:
    Goalpose(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    void publishGoalPose(double length, double angle);

    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_turning;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriptionpose;

    double posex{0.0}, posey{0.0};
    double orientationw{0.0}, orientationx{0.0}, orientationy{0.0}, orientationz{0.0};
};
