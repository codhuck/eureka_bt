#include <rclcpp/rclcpp.hpp>
#include <eureka_bt/cv.hpp>
#include <eureka_bt/goal_pose.hpp>
#include <eureka_bt/turn_inside.hpp>
#include <behaviortree_cpp_v3/bt_factory.h> 
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/control_node.h>
#include <eureka_bt/bt_action_node.hpp>


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("main_bt");

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<CV_detection>("CV_detection");
  factory.registerNodeType<Goalpose>("Goal_pose");  
  factory.registerNodeType<Turn_inside>("Turn_inside");

  auto tree = factory.createTreeFromFile("./src/eureka_bt/xml_tree/tree.xml");

  BT::NodeConfiguration con = {};
  auto lc_listener = std::make_shared<CV_detection>("lc_listener", con);
  auto lc_goal = std::make_shared<Goalpose>("lc_goal", con);
  auto lc_turn = std::make_shared<Turn_inside>("lc_turn", con);

  rclcpp::Rate rate(10); 

  while (rclcpp::ok()) {
    rclcpp::spin_some(lc_listener);
    rclcpp::spin_some(lc_goal);
    rclcpp::spin_some(lc_turn); 
    tree.tickRoot();
  }

  rclcpp::shutdown();
  return 0;
}

