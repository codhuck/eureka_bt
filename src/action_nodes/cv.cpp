#include "eureka_bt/cv.hpp"

CV_detection::CV_detection(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config),  Node("CV_detection") {
    subscription = this->create_subscription<sensor_msgs::msg::JointState>(
        "/arrow_detection", 10,
        [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
            std::cout << "tick" << std::endl;
            if (!msg->name.empty()) {
                names_.emplace_back(msg->name[0]);
                positions_.emplace_back(msg->position[0]);
                velocities_.emplace_back(msg->velocity[0]);
                efforts_.emplace_back(msg->effort[0]);

                if (names_.size() == 10) {
                    processValues(); 
                    clearData();    
                }
            }
        });
}


BT::PortsList CV_detection::providedPorts() {
    return {
        BT::OutputPort<std::string>("narrow_arrow"),
        BT::OutputPort<double>("length"),
        BT::OutputPort<double>("angle"),
        BT::OutputPort<double>("coef")
    };
}

BT::NodeStatus CV_detection::tick() {
    setOutput("narrow_arrow", narrow);
    setOutput("length", length);
    setOutput("angle", angle);
    setOutput("coef", coef);
    return BT::NodeStatus::SUCCESS;
}

void CV_detection::processValues() {
    bool has_no_detection = std::any_of(names_.begin(), names_.end(), [](const std::string& name) {
        return name == "No_detection";
    });

    narrow = has_no_detection ? "No_detection" : names_.back();

    length = calculateAverage(positions_);
    angle = calculateAverage(velocities_);
    coef = calculateAverage(efforts_);
}

double CV_detection::calculateAverage(const std::vector<double>& values) {
    double sum = 0.0;
    for (const auto& value : values) {
        sum += value;
    }
    return sum / values.size();  
}

void CV_detection::clearData() {
    names_.clear();
    positions_.clear();
    velocities_.clear();
    efforts_.clear();
}
