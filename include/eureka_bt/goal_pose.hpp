#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <eureka_bt/bt_action_node.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <chrono>
#include <thread>
#include <nav_msgs/msg/occupancy_grid.hpp>

class Goalpose : public BT::SyncActionNode, public rclcpp::Node {
public:
Goalpose(const std::string& name, const BT::NodeConfiguration& config);

static BT::PortsList providedPorts();

BT::NodeStatus tick() override;

private:
void publishGoalPose(double length, double angle);
void moveForward();
void rotate(double angle_deg);
void processCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
bool isObstacle(double x, double y);

rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriptionpose;
rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_costmap;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_turning;
};