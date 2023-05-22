#include <cstdio>
#include <memory>
#include <chrono>

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


#define SPIN

#ifdef SPIN
     //spin the node
     //rclcpp::executors::SingleThreadedExecutor executor;
     rclcpp::executors::SingleThreadedExecutor executor;
	   printf("ok here two\n");
     executor.add_node(node);
	   printf("ok here three\n");

     auto spinner = std::thread([&executor]() { executor.spin(); });

#endif




#define LKIN

#ifdef LKIN 
	   printf("ok here 1\n");
   // get all kinematics parameters from the move_group node
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
	   printf("ok here 2\n");
    auto plist = parameters_client->load_parameters("kinematics.yaml",  std::chrono::seconds(1)  );

    //printf("%s\n", plist.parameters );
     /*
    while (!parameters_client->wait_for_service( std::chrono::seconds(1) )) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
#ifdef SPIN
        spinner.join();
#endif	
      }
      RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }
    */



	   printf("ok here 3\n");
    //executor.remove_node(node);
    rcl_interfaces::msg::ListParametersResult parameter_list = parameters_client->list_parameters({"robot_description_kinematics"},15);
	   printf("ok here 3 1\n");
    //printf("names %s\n", (char *) parameter_list.names );

    //auto parameters = parameters_client->get_parameters(parameter_list.names);
    //printf("names %s\n",  (char *) parameter_list.names );
    // set the parameters to our node
	   printf("ok here 4\n");
    //node->set_parameters(parameters);
	   printf("ok here 5\n");
#endif





  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");
  //auto move_group_interface = MoveGroupInterface(node, "panda_arm");


  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.434;
    msg.orientation.y = 0.237;
    msg.orientation.z = 0.417;
    msg.orientation.w = 0.763;
    msg.position.x = -1.514;
    msg.position.y =  0.806;
    msg.position.z =  3.206;
    return msg;
  }();

   auto const ptarget_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();


  /*
   moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
   move_group_interface.setStartState(start_state);
  //const moveit::core::JointModelGroup* joint_model_group =
  //  move_group.getCurrentState()->getJointModelGroup("arm");
  // start_state.setFromIK(joint_model_group, start_pose2);
  //move_group_interface.setStartState(target_pose);

   */

//  for (int i=0; i < 100000; i++ );


  printf("Before set target \n");
  //move_group_interface.setPoseTarget(target_pose);
  //move_group_interface.setPoseTarget(target_pose, "eef_link");
  //move_group_interface.setApproximateJointValueTarget(target_pose, "panda_link8");
  move_group_interface.setApproximateJointValueTarget(target_pose);
  //move_group_interface.setRPYTarget(1.5,0.017, 0.017); 
  //move_group_interface.setNamedTarget("pose1");
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

#ifdef SPIN
    spinner.join();
#endif 

  return 0;
}
