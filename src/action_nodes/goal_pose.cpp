#include "eureka_bt/goal_pose.hpp"

    double posex, posey;
    double orientationw, orientationx, orientationy, orientationz;

Goalpose::Goalpose(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), Node("Goal_pose") {

    subscriptionpose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/localization_pose", 10,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg) {
            posex = msg->pose.pose.position.x;
            posey = msg->pose.pose.position.y;
            orientationw = msg->pose.pose.orientation.w;
            orientationx = msg->pose.pose.orientation.x;
            orientationy = msg->pose.pose.orientation.y;
            orientationz = msg->pose.pose.orientation.z;
        }
    );

    publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    publisher_turning = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

BT::PortsList Goalpose::providedPorts() {
    return {
        BT::InputPort<std::string>("narrow_arrow"),
        BT::InputPort<double>("length"),
        BT::InputPort<double>("angle"),
        BT::InputPort<double>("coef"),
        BT::InputPort<bool>("turning_koef")
    };
}

BT::NodeStatus Goalpose::tick() {
    auto msgnarrow = getInput<std::string>("narrow_arrow");
    auto msglength = getInput<double>("length");
    auto angle = getInput<double>("angle");
    auto coef = getInput<double>("coef");
    auto turning_koef = getInput<bool>("turning_koef");

    if (*msglength > 1.8 && *msgnarrow != "No_detection" && *turning_koef == false && *coef > 0.5) {
        publishGoalPose(*msglength, *angle);
    }

    return BT::NodeStatus::SUCCESS;
}

void Goalpose::publishGoalPose(double length, double angle) 
{
    geometry_msgs::msg::PoseStamped goalposemsg;
    goalposemsg.header.stamp = this->now();
    goalposemsg.header.frame_id = "map"; 
    double yaw_sh = atan2(2.0 * (orientationw * orientationz + orientationx * orientationy),
                       1.0 - 2.0 * (orientationy * orientationy + orientationz * orientationz));
    double localx = (length - 1.0);
    double localy = (length - 1.0) * sin(-angle); 
    double globalx = posex + (localx * cos(yaw_sh) - localy * sin(yaw_sh));
    double globaly = posey + (localx * sin(yaw_sh) + localy * cos(yaw_sh));
    goalposemsg.pose.position.x = globalx;
    goalposemsg.pose.position.y = globaly;
    goalposemsg.pose.position.z = 0.0;
    goalposemsg.pose.orientation.x = orientationx;
    goalposemsg.pose.orientation.y = orientationy;
    goalposemsg.pose.orientation.z = orientationz;
    goalposemsg.pose.orientation.w = orientationw;
    double yaw = (atan2(2.0 * (orientationw * orientationz + orientationx * orientationy), 1.0 - 2.0 * (orientationy * orientationy + orientationz * orientationz))) * (180.0/M_PI);


    publisher->publish(goalposemsg);
    while (posex > globalx + 0.5 && posex < globalx - 0.5 && posey > globaly + 0.5 && posey < globaly - 0.5 && yaw<yaw_sh-4.0 && yaw>yaw_sh+4.0) 
    {          
        yaw = (atan2(2.0 * (orientationw * orientationz + orientationx * orientationy), 1.0 - 2.0 * (orientationy * orientationy + orientationz * orientationz))) * (180.0/M_PI);  
    }
}

