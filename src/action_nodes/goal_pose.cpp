#include "eureka_bt/goal_pose.hpp"

Goalpose::Goalpose(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), Node("Goal_pose") {

    subscriptionpose = this->create_subscription<geometry_msgs::msg::Pose>(
        "/odometry", 10,
        [this](geometry_msgs::msg::Pose::UniquePtr msg) {
            posex = msg->position.x;
            posey = msg->position.y;
            orientationw = msg->orientation.w;
            orientationx = msg->orientation.x;
            orientationy = msg->orientation.y;
            orientationz = msg->orientation.z;
        }
    );

    publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
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

    if (*msglength > 1.8 && *msgnarrow != "No_detection" && *turning_koef == false) {
        publishGoalPose(*msglength, *angle);
    }

    return BT::NodeStatus::SUCCESS;
}

void Goalpose::publishGoalPose(double length, double angle) {
    geometry_msgs::msg::PoseStamped goalposemsg;
    goalposemsg.header.stamp = this->now();
    goalposemsg.header.frame_id = "map"; 

    double yaw = atan2(2.0 * (orientationw * orientationz + orientationx * orientationy),
                       1.0 - 2.0 * (orientationy * orientationy + orientationz * orientationz));

    double localx = (length - 1.0);
    double localy = (length - 1.0) * sin(-angle); 

    double globalx = posex + (localx * cos(yaw) - localy * sin(yaw));
    double globaly = posey + (localx * sin(yaw) + localy * cos(yaw));

    goalposemsg.pose.position.x = globalx;
    goalposemsg.pose.position.y = globaly;
    goalposemsg.pose.position.z = 0.0;
    goalposemsg.pose.orientation.x = orientationx;
    goalposemsg.pose.orientation.y = orientationy;
    goalposemsg.pose.orientation.z = orientationz;
    goalposemsg.pose.orientation.w = orientationw;

    publisher->publish(goalposemsg);

    while (posex > globalx + 1 || posex < globalx - 1 || posey > globaly + 1 || posey < globaly - 1) 
    {  }
}