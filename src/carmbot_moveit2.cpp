#include <cstdio>
#include <memory>
#include <chrono>
#include <vector>


#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;

auto const logger = rclcpp::get_logger("carmbot_moveit2");
   

void tprint( moveit::planning_interface::MoveGroupInterface::Plan &plan ) {

  auto msg = plan.trajectory_;

  std::vector<std::string> jnames = plan.trajectory_.joint_trajectory.joint_names ;

  std::vector<float>::size_type tsize = msg.joint_trajectory.points.size() ;

  printf("size of trajectory: %ld\n", tsize );

  std::vector<double> tpos ;
  for (unsigned i=0; i< tsize; i++)
  {
     tpos = msg.joint_trajectory.points[i].positions;
     printf("position %d = %f %f %f\n", i, tpos[0], tpos[1], tpos[2] );
  }

}



int plan_move ( moveit::planning_interface::MoveGroupInterface &move_group_interface,
		std::vector<float> &pos, std::vector<float> &ang) {

  int rval = 0;

  // Set a target Pose
  auto target_pose = [&pos, &ang]{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = ang[0];
    msg.orientation.y = ang[1];
    msg.orientation.z = ang[2];
    msg.orientation.w = ang[3];
    msg.position.x = pos[0];
    msg.position.y = pos[1];
    msg.position.z = pos[2];
    return msg;
  }();

  printf("target position(xyz) %f %f %f\n", target_pose.position.x, target_pose.position.y, target_pose.position.z);

  //moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
  //move_group_interface.setStartState(start_state);



  move_group_interface.setApproximateJointValueTarget(target_pose);

  // Create a plan to that target pose
  auto  [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  tprint( plan);

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
    rval = -1;
  }

  return rval;
}




int main(int argc, char ** argv)
{

  printf("Starting carmbot_moveit2 package\n");


  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "carmbot_moveit2",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  

#define SPIN

#ifdef SPIN
     //spin the node
     rclcpp::executors::SingleThreadedExecutor executor;
	   printf("ok here two\n");
     executor.add_node(node);
	   printf("ok here three\n");
     auto spinner = std::thread([&executor]() { executor.spin(); });
#endif

 
  // Create the MoveIt MoveGroup Interface
  auto move_group_interface = MoveGroupInterface(node, "arm");
  

   std::vector<float> pos{-1.514, 0.806, 3.206}; std::vector<float> ang{0.434, 0.237, 0.417, 0.763};
   plan_move (move_group_interface, pos, ang );
  
   std::vector<float> pos1{0.192, 3.065, 0.852}; std::vector<float> ang1{0.026, 0.000, 0.008, 1.0};
   plan_move (move_group_interface, pos1, ang1 );
 
   // Shutdown ROS
  rclcpp::shutdown();

#ifdef SPIN
    spinner.join();
#endif 

  return 0;
}
