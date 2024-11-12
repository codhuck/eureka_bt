#include <string>
#include <vector>
#include <sensor_msgs/msg/joint_state.hpp>
#include <behaviortree_cpp_v3/bt_factory.h> 
#include "rclcpp/rclcpp.hpp"

class CV_detection : public BT::SyncActionNode, public rclcpp::Node {
public:
    CV_detection(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    void processValues();
    double calculateAverage(const std::vector<double>& values);
    void clearData();

    std::vector<std::string> names_;
    std::vector<double> positions_;
    std::vector<double> velocities_;
    std::vector<double> efforts_;

    std::string narrow = "No_detection";
    double length = 0.0;
    double angle = 0.0;
    double coef = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription;
    rclcpp::Node::SharedPtr node;

};
