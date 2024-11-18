#include "eureka_bt/turn_inside.hpp"
    double pose_x_;
    double pose_y_;
    double orientationw_;
    double orientationx_;
    double orientationy_;
    double orientationz_;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

Turn_inside::Turn_inside(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), Node("turn_inside")
{
    subscribe_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/localization_pose", 10,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg) {
            pose_x_ = msg->pose.pose.position.x;
            pose_y_ = msg->pose.pose.position.y;
            orientationw_ = msg->pose.pose.orientation.w;
            orientationx_ = msg->pose.pose.orientation.x;
            orientationy_ = msg->pose.pose.orientation.y;
            orientationz_ = msg->pose.pose.orientation.z;
        }
    );

    publisher_turn = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    action_client = rclcpp_action::create_client<NavigateToPose>(
        this,
        "navigate_to_pose");
}

BT::PortsList Turn_inside::providedPorts() 
{
    return {
        BT::InputPort<std::string>("narrow_arrow"),
        BT::InputPort<double>("length"),
        BT::InputPort<double>("angle"),
        BT::InputPort<double>("coef"),
        BT::OutputPort<bool>("turning_koef")
    };
}

BT::NodeStatus Turn_inside::tick() 
{
    std::string narrow_arrow;
    double length;
    double coef;

    if (!getInput<std::string>("narrow_arrow", narrow_arrow) 
        || !getInput<double>("length", length) 
        || !getInput<double>("coef", coef)) 
    {
        return BT::NodeStatus::FAILURE;
    }
    
    if (length > 0.0 && length < 1.8 && coef > 0.7) 
    {
        setOutput("turning_koef", true);
        auto future_cancel = action_client->async_cancel_all_goals();
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;
        publisher_turn->publish(twist_msg);
        std::this_thread::sleep_for(std::chrono::seconds(12));
        double turn_angle = (narrow_arrow == "left") ? 90.0 : -90.0;
        updateGoalPose(turn_angle);
    } 
    else 
    {
        setOutput("turning_koef", false);
    }
    return BT::NodeStatus::FAILURE; 
}

void Turn_inside::updateGoalPose(double turn_angle)
{
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 100.0;
    if (turn_angle > 0)
    {
        twist_msg.linear.x = 50.0;
    }
    else
    {
        twist_msg.linear.x = -50.0;
    }

    double yaw_sh = atan2(2.0 * (orientationw_ * orientationz_ + orientationx_ * orientationy_),
                       1.0 - 2.0 * (orientationy_ * orientationy_ + orientationz_ * orientationz_)) + (turn_angle * M_PI / 180.0);
    
    publisher_turn->publish(twist_msg);
    double yaw = (atan2(2.0 * (orientationw_ * orientationz_ + orientationx_ * orientationy_), 1.0 - 2.0 * (orientationy_ * orientationy_ + orientationz_ * orientationz_))) * (180.0/M_PI);
    while (yaw<yaw_sh-4.0 && yaw>yaw_sh+4.0)
    {

        yaw = (atan2(2.0 * (orientationw_ * orientationz_ + orientationx_ * orientationy_), 1.0 - 2.0 * (orientationy_ * orientationy_ + orientationz_ * orientationz_))) * (180.0/M_PI);  
    }
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;
    publisher_turn->publish(twist_msg);
}

