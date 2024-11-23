#include "eureka_bt/turn_inside.hpp"
    double pose_x_;
    double pose_y_;
    double orientationw_;
    double orientationx_;
    double orientationy_;
    double orientationz_;
        double coef_for_turning = 0.0;
    double value_of_turn;

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

   if (length > 0.0 && length < 2.1 && coef > 0.7 && narrow_arrow != "No_detection" && coef_for_turning == 0.0) 
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
        std::cout<< "Time_for_waiting" << std::endl;
        double turn_angle = (narrow_arrow == "left") ? 90.0 : -90.0;
        updateGoalPose(turn_angle);
    } 
    if (coef_for_turning == 1.0) 
    {
        tf2::Quaternion quaternion(
            orientationx_,
            orientationy_,
            orientationz_,
            orientationw_);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        if (yaw < value_of_turn + 0.1 && yaw > value_of_turn - 0.1)
        {
            coef_for_turning = 0.0;
            geometry_msgs::msg::Twist twist_msg;
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;
            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = 0.0;
            publisher_turn->publish(twist_msg);
            setOutput("turning_koef", false);
        }
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
        twist_msg.linear.x = 0.14;
    }
    else
    {
        twist_msg.linear.x = -0.14;
    }
            tf2::Quaternion quaternion(
            orientationx_,
            orientationy_,
            orientationz_,
            orientationw_);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        yaw=yaw + M_PI_2;
        while (yaw > M_PI)
        {
            yaw -= 2.0 * M_PI;
        }
        while (yaw < -M_PI)
        {
            yaw += 2.0 * M_PI;
        }
        value_of_turn = yaw;
    publisher_turn->publish(twist_msg);
    coef_for_turning = 1.0;
}