#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string.h>
#include <map>


// Currently this bug exists. I am saving its problem for future me:
//      if this executable is run before the one that moves the arm, then obstacles will not be spawned into the environment
//      thus if this is run first, gripper movements will not consider the enviornment. Which is bad. 


int main(int argc, char** argv)
{

  const int NUM_ARGS = 2;

  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "ur5e_mover_gripper_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // We use a single-threaded executor for simplicity
  rclcpp::executors::SingleThreadedExecutor executor; // SingleThreadedExecutor
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // 1. Setup the MoveGroupInterface for the "ur_arm" planning group
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "gripper_ee");


  RCLCPP_INFO(node->get_logger(), "The current end effector link is ");
  RCLCPP_INFO(node->get_logger(), move_group_interface.getEndEffectorLink().c_str());


  auto link_names = move_group_interface.getLinkNames();

  for (auto iter = link_names.begin(); iter != link_names.end(); iter++) {
    RCLCPP_INFO(node->get_logger(), (*iter).c_str());
  }

  // // 1. Get the current pose of the end-effector (usually "tool0")
  // // auto live_pose = move_group_interface.getPoseTarget(); 
  // // Note: Use move_group_interface.getCurrentPose() for the live state
  // auto live_pose = move_group_interface.getCurrentPose("ur5e_gripper_tcp");
//   auto live_pose = move_group_interface.getCurrentPose("ur5e_base_link");



  if (argc != NUM_ARGS) {
    // const char* wrong_num_args_msg = ("Incorrect number of arguments! Expected " + std::to_string(NUM_ARGS) + " but got " + std::to_string(argc) + "!").c_str();
    RCLCPP_ERROR(node->get_logger(), "bad num args. Should receive one argument, either the desired decimal position, or 'open' or 'close' (not in quotes)");
    // TODO: Make this error message better.
    rclcpp::shutdown();
    return 1;
  } 

  for (int i = 0; i < argc; i++) {
    RCLCPP_INFO(node->get_logger(), argv[i]);
  }



  // 2. Define the target position (decimal value)
  // 0.0 is usually fully closed, while larger values are open

  std::string target_str = argv[1];
  double target_position;
  const double MAX_TARGET_POS = 0.59;
  const double MIN_TARGET_POS = 0.01;
  if (target_str == "open") {
    target_position = 0.05;
  } else if (target_str == "close") {
    target_position = 0.55;
  } else {
    target_position = std::stod(target_str);
    if (target_position > MAX_TARGET_POS || target_position < MIN_TARGET_POS) {
        RCLCPP_ERROR(node->get_logger(), "Received a target position input outside of the safe range (0.01, 0.59)!");
        rclcpp::shutdown();
        return 1;
    }
  }

  // 3. Set the joint value target
  // "finger_joint" must match the joint name in your URDF
  std::map<std::string, double> joint_targets;
  joint_targets["ur5e_gripper_finger_joint"] = target_position;
  
  move_group_interface.setJointValueTarget(joint_targets);



  move_group_interface.setNumPlanningAttempts(25); // why not. It's taking usually like 0.05 sec per plan



  // // 3. Create a plan
  MoveGroupInterface::Plan my_plan;

  rclcpp::sleep_for(std::chrono::seconds(1));


  bool success = move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;


  // 4. Execute the plan
  if (success) {
    move_group_interface.execute(my_plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed!");
  }

  // comment out the below section for not cartesian



  rclcpp::shutdown();
  return 0;
}