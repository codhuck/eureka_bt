#include <behaviortree_cpp_v3/bt_factory.h> 
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class Goalpose : public BT::SyncActionNode, public rclcpp::Node {
public:
    Goalpose(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    void publishGoalPose(double length, double angle);

    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriptionpose;

    double posex{0.0}, posey{0.0};
    double orientationw{0.0}, orientationx{0.0}, orientationy{0.0}, orientationz{0.0};
};
