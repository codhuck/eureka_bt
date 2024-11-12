#include "eureka_bt/turn_inside.hpp"

Turn_inside::Turn_inside(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config),
      Node("turn_inside")
{
    subscribe_pose_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/odometry", 10,
        [this](geometry_msgs::msg::Pose::UniquePtr msg) {
            pose_x_ = msg->position.x;
            pose_y_ = msg->position.y;
            orientationw = msg->orientation.w;
            orientationx = msg->orientation.x;
            orientationy = msg->orientation.y;
            orientationz = msg->orientation.z;
        }
    );

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
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

    if (!getInput<std::string>("narrow_arrow", narrow_arrow) ||
        !getInput<double>("length", length)|| 
        !getInput<double>("coef", coef)) 
    {
        return BT::NodeStatus::FAILURE;
    }
    if (length>0 && length < 1.8 && coef > 0.7) {
        setOutput("turning_koef", true);
        std::this_thread::sleep_for(std::chrono::seconds(12));
        double turn_angle = (narrow_arrow == "left") ? 90.0 : -90.0;
        updateGoalPose(turn_angle);
        return BT::NodeStatus::RUNNING; 
    }
    else 
    {
        setOutput("turning_koef", false);
    }
    return BT::NodeStatus::FAILURE; 
}

void Turn_inside::updateGoalPose(double turn_angle)
{
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = this->now();
    goal_msg.header.frame_id = "map";

    double yaw_sh = atan2(2.0 * (orientationw * orientationz + orientationx * orientationy),
                       1.0 - 2.0 * (orientationy * orientationy + orientationz * orientationz)) + (turn_angle * M_PI / 180.0);
    goal_msg.pose.position.x = pose_x_;
    goal_msg.pose.position.y = pose_y_;
    goal_msg.pose.position.z = 0.0; 

    goal_msg.pose.orientation.z = std::sin(yaw_sh / 2.0);
    goal_msg.pose.orientation.w = std::cos(yaw_sh / 2.0);
    
    publisher_->publish(goal_msg);
    double yaw = (atan2(2.0 * (orientationw * orientationz + orientationx * orientationy), 1.0 - 2.0 * (orientationy * orientationy + orientationz)))* (180.0/M_PI);
    while (yaw<yaw_sh-4.0 || yaw>yaw_sh+4.0)
    {  
        yaw = (atan2(2.0 * (orientationw * orientationz + orientationx * orientationy), 1.0 - 2.0 * (orientationy * orientationy + orientationz * orientationz)))* (180.0/M_PI);  
    }
}
