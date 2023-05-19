#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char ** argv)
{

  printf("Starting carmbot_moveit2 package\n");

  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "carmbot_moveit2",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("carmbot_moveit2");



  //spin the node
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  /*
  */



  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    //msg.orientation.w = 0.0;
    msg.position.x = 0.11;
    msg.position.y = 1.25;
    msg.position.z = 0.0;
    return msg;
  }();



   moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
   move_group_interface.setStartState(start_state);
  //const moveit::core::JointModelGroup* joint_model_group =
  //  move_group.getCurrentState()->getJointModelGroup("arm");
  // start_state.setFromIK(joint_model_group, start_pose2);
  //move_group_interface.setStartState(target_pose);

  /*
   */



  printf("Before set target \n");
  //move_group_interface.setPoseTarget(target_pose, "eef_link");
  //move_group_interface.setApproximateJointValueTarget(target_pose, "eef_link");
  //move_group_interface.setApproximateJointValueTarget(target_pose);
  //move_group_interface.setRPYTarget(0.17,0.0, 0); 
  move_group_interface.setNamedTarget("pose1");
  printf("After set target \n");

  
  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  
  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }
  
  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();

  return 0;
}
