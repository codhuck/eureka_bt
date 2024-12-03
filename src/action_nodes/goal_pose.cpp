#include "eureka_bt/goal_pose.hpp"

double posex, posey;
double orientationw, orientationx, orientationy, orientationz;
double coef_goal_pose = 0.0;
std::vector<std::vector<int>> global_costmap;

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

    subscription_costmap = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", 10, 
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            processCostmap(msg);
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

void Goalpose::processCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    global_costmap.clear();
    int width = msg->info.width;
    int height = msg->info.height;
    global_costmap.resize(height, std::vector<int>(width, 0));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            global_costmap[y][x] = msg->data[y * width + x];
        }
    }
}

bool Goalpose::isObstacle(double x, double y) {
    if (global_costmap.empty()) return false;

    int map_x = static_cast<int>((x - posex) / 0.05); 
    int map_y = static_cast<int>((y - posey) / 0.05);

    if (map_x < 0 || map_y  < 0 || map_y >= global_costmap.size() || map_x >= global_costmap[0].size()) 
    {
        return true;
    }
    return global_costmap[map_y][map_x] > 0;
}

void Goalpose::rotate(double angle_deg) {
    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = angle_deg > 0 ? 0.5 : -0.5; 

    double rotation_time = std::abs(angle_deg / 30.0); 
    auto start_time = std::chrono::steady_clock::now();

    while (std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::steady_clock::now() - start_time)
               .count() < rotation_time) {
        publisher_turning->publish(cmd_msg);
    }

    cmd_msg.angular.z = 0.0; 
    publisher_turning->publish(cmd_msg);
}

BT::NodeStatus Goalpose::tick() {
    auto msgnarrow = getInput<std::string>("narrow_arrow");
    auto msglength = getInput<double>("length");
    auto angle = getInput<double>("angle");
    auto coef = getInput<double>("coef");
    auto turning_koef = getInput<bool>("turning_koef");

//   if (*msgnarrow == "No_detection" &&  coef_goal_pose == 0.0) {
  //      publishGoalPose(2.0, 0.0);
    //    rotate(30.0); 
      //  rotate(-30.0); 
        //return BT::NodeStatus::SUCCESS;
    //}

    if (*msglength > 1.0 && *msgnarrow != "No_detection" && *coef > 0.6) {
        std::cout<<"Goal"<<std::endl;
        publishGoalPose(*msglength, *angle);
    }

    return BT::NodeStatus::SUCCESS;
}

void Goalpose::publishGoalPose(double length, double angle) {
    geometry_msgs::msg::PoseStamped goalposemsg;
    goalposemsg.header.stamp = this->now();
    goalposemsg.header.frame_id = "map";

    double yaw_sh = atan2(2.0 * (orientationw * orientationz + orientationx * orientationy),
                          1.0 - 2.0 * (orientationy * orientationy + orientationz * orientationz));
    double localx = (length )* cos(-angle * (M_PI / 180.0));
    double localy = (length) * sin(-angle * (M_PI / 180.0));
    double globalx = posex + (localx * cos(yaw_sh) - localy * sin(yaw_sh));
    double globaly = posey + (localx * sin(yaw_sh) + localy * cos(yaw_sh));

    if (isObstacle(globalx, globaly)) {
        for (double offset = 0.1; offset <= 1.0; offset += 0.1) {
            for (double angle_offset = -M_PI; angle_offset <= M_PI; angle_offset += M_PI / 6) {
                double new_x = posex + offset * cos(angle_offset);
                double new_y = posey + offset * sin(angle_offset);
                if (!isObstacle(new_x, new_y)) {
                    globalx = new_x;
                    globaly = new_y;
                    goto goal_found;
                }
            }
        }
    goal_found:;
    }

    goalposemsg.pose.position.x = globalx;
    goalposemsg.pose.position.y = globaly;
    goalposemsg.pose.position.z = 0.0;
    goalposemsg.pose.orientation.x = orientationx;
    goalposemsg.pose.orientation.y = orientationy;
    goalposemsg.pose.orientation.z = orientationz;
    goalposemsg.pose.orientation.w = orientationw;

    if (coef_goal_pose == 0.0) {
        publisher->publish(goalposemsg);
    }

    if (posex > globalx + 0.05 || posex < globalx - 0.05 || posey > globaly + 0.05 || posey < globaly - 0.05) {
        coef_goal_pose = 1.0;
    } else {
        coef_goal_pose = 0.0;
    }
}