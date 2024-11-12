#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class Turn_inside : public BT::SyncActionNode, public rclcpp::Node {
public:
    Turn_inside(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:

    void updateGoalPose(double turn_angle);

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscribe_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_turn;
    rclcpp::TimerBase::SharedPtr timer_;
    double pose_x_;
    double pose_y_;
    double orientationx;
    double orientationy;
    double orientationw;
    double orientationz;
};