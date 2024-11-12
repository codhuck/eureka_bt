#include <rclcpp/rclcpp.hpp>
#include <eureka_bt/cv.hpp>
#include <eureka_bt/goal_pose.hpp>
#include <eureka_bt/turn_inside.hpp>
#include <behaviortree_cpp_v3/bt_factory.h> 
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/control_node.h>

static const char *xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Parallel name="root_parallel" success_threshold="5" failure_threshold="5">
            <CV_detection name="CV_detection" narrow_arrow="{narrow_arrow}" length="{length}" angle="{angle}" coef="{coef}"/>
            <Goal_pose name="Goal_pose" narrow_arrow="{narrow_arrow}" length="{length}" angle="{angle}" coef="{coef}" turning_koef="{turning_koef}"/>
            <Turn_inside name="Turn_inside" narrow_arrow="{narrow_arrow}" length="{length}" angle="{angle}" coef="{coef}" turning_koef="{turning_koef}"/>
        </Parallel>
    </BehaviorTree>
</root>
 )";

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("main_bt");

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<CV_detection>("CV_detection");
  factory.registerNodeType<Goalpose>("Goal_pose");  
  factory.registerNodeType<Turn_inside>("Turn_inside");

  auto tree = factory.createTreeFromText(xml_text);

  BT::NodeConfiguration con = {};
  auto lc_listener = std::make_shared<CV_detection>("lc_listener", con);
  auto lc_odom = std::make_shared<Goalpose>("lc_odom", con);
  auto lc_odomi = std::make_shared<Turn_inside>("lc_odomi", con);

  rclcpp::Rate rate(10); 

  while (rclcpp::ok()) {
    rclcpp::spin_some(lc_listener);
    rclcpp::spin_some(lc_odom);
    rclcpp::spin_some(lc_odomi);
    tree.sleep(std::chrono::milliseconds(1));
  }

  rclcpp::shutdown();
  return 0;
}

