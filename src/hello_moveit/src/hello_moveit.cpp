#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string.h>


int main(int argc, char** argv)
{

  const int NUM_ARGS = 7;

  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "ur_moveit_cpp_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // We use a single-threaded executor for simplicity
  rclcpp::executors::SingleThreadedExecutor executor; // SingleThreadedExecutor
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // 1. Setup the MoveGroupInterface for the "ur_arm" planning group
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_arm");


  if (argc == NUM_ARGS + 1) {
    std::string desired_ee = argv[NUM_ARGS];
    
    if (desired_ee == "tcp") {
      move_group_interface.setEndEffectorLink("ur5e_gripper_tcp");
    } else if (desired_ee == "pre_grasp") {
      move_group_interface.setEndEffectorLink("ur5e_gripper_pre_grasp");
    } else {
      RCLCPP_ERROR(node->get_logger(), "Got a desired end effector that is not 'tcp' or 'pre_grasp'. Aborting for safety");
      rclcpp::shutdown();
      return 1;
    }
    
    // std::string info_
    RCLCPP_INFO(node->get_logger(), ("Set the end effector to " + move_group_interface.getEndEffectorLink()).c_str());

  } else if (argc != NUM_ARGS) {
    // const char* wrong_num_args_msg = ("Incorrect number of arguments! Expected " + std::to_string(NUM_ARGS) + " but got " + std::to_string(argc) + "!").c_str();
    RCLCPP_ERROR(node->get_logger(), "bad num args. Should receive: x y z yaw pitch roll");
    // TODO: Make this error message better.
    rclcpp::shutdown();
    return 1;
  } 

  for (int i = 0; i < argc; i++) {
    RCLCPP_INFO(node->get_logger(), argv[i]);
  }

  RCLCPP_INFO(node->get_logger(), ("The current end effector link is " + move_group_interface.getEndEffectorLink()).c_str());

  auto link_names = move_group_interface.getLinkNames();

  for (auto iter = link_names.begin(); iter != link_names.end(); iter++) {
    RCLCPP_INFO(node->get_logger(), (*iter).c_str());
  }

  // // 1. Get the current pose of the end-effector (usually "tool0")
  // // auto live_pose = move_group_interface.getPoseTarget(); 
  // // Note: Use move_group_interface.getCurrentPose() for the live state
  // auto live_pose = move_group_interface.getCurrentPose("ur5e_tool0"); // "ur5e_gripper_tcp"
  auto live_pose = move_group_interface.getCurrentPose(move_group_interface.getEndEffectorLink());


  tf2::Quaternion curr_q(live_pose.pose.orientation.x, live_pose.pose.orientation.y, live_pose.pose.orientation.z, live_pose.pose.orientation.w);
  curr_q.normalize();
  tf2::Matrix3x3 curr_euler(curr_q);
  double curr_roll, curr_pitch, curr_yaw;
  curr_euler.getRPY(curr_roll, curr_pitch, curr_yaw);

  RCLCPP_INFO(node->get_logger(), "Current Quaternion: x=%f, y=%f, z=%f, w=%f",
          curr_q.getX(),
          curr_q.getY(),
          curr_q.getZ(),
          curr_q.getW());

  // 2. Extract and print Position
  RCLCPP_INFO(node->get_logger(), "Current Position: x=%f, y=%f, z=%f",
              live_pose.pose.position.x,
              live_pose.pose.position.y,
              live_pose.pose.position.z);

  // 3. Extract and print Quaternion
  RCLCPP_INFO(node->get_logger(), "Current Orientation: yaw=%f, pitch=%f, roll=%f",
              curr_yaw * 180 / M_PI,
              curr_pitch * 180 / M_PI,
              curr_roll * 180 / M_PI);


  // 2. Set a target Pose (values in meters/radians)
  geometry_msgs::msg::Pose target_pose;

  const char* do_nothing_str = "x";

  // 1. Define your angles in RADIANS

  double roll = (!strcmp(do_nothing_str, argv[6])) ? curr_roll : std::stod(argv[6]) * M_PI / 180.0;
  double pitch = (!strcmp(do_nothing_str, argv[5])) ? curr_pitch : std::stod(argv[5]) * M_PI / 180.0;
  double yaw = (!strcmp(do_nothing_str, argv[4])) ? curr_yaw : std::stod(argv[4]) * M_PI / 180.0;

  // 2. Create a tf2 Quaternion and set it using Euler angles
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  // 3. Convert to the message format used by MoveIt
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();

  // target_pose.orientation.w = 1.0;

  // singularities from roll pitch yaw bad
  // quaternion fixes
  // there is a function for this

  target_pose.position.x = (!strcmp(do_nothing_str, argv[1])) ? live_pose.pose.position.x : std::stod(argv[1]);
  target_pose.position.y = (!strcmp(do_nothing_str, argv[2])) ? live_pose.pose.position.y : std::stod(argv[2]);
  target_pose.position.z = (!strcmp(do_nothing_str, argv[3])) ? live_pose.pose.position.z : std::stod(argv[3]);

  // target_pose.position.x = std::stod(argv[1]);
  // target_pose.position.y = std::stod(argv[2]);
  // target_pose.position.z = std::stod(argv[3]);




  // move_group_interface.setEndEffectorLink("ur5e_tool0"); // the default should be "ur5e_gripper_pre_grasp"




  RCLCPP_INFO(node->get_logger(), "Goal Quaternion: x=%f, y=%f, z=%f, w=%f",
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w);          


  move_group_interface.setPoseTarget(target_pose);

  // std::vector<double> joint_group_positions = {
  //   0.0,    // ur5e_shoulder_pan_joint
  //   -1.57,  // ur5e_shoulder_lift_joint (bent forward)
  //   0.0,   // ur5e_elbow_joint (90 degrees up)
  //   -1.57,  // ur5e_wrist_1_joint (level)
  //   -1.57,  // ur5e_wrist_2_joint (rotated)
  //   0.0     // ur5e_wrist_3_joint
  // };

  // move_group_interface.setJointValueTarget(joint_group_positions);



  // // 3. Create a plan
  MoveGroupInterface::Plan my_plan;

  
    // Create collision object for the robot to avoid
  auto const the_floor = [frame_id =
                                  move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject the_floor;
    the_floor.header.frame_id = frame_id;
    the_floor.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.5;
    primitive.dimensions[primitive.BOX_Y] = 1.5;
    primitive.dimensions[primitive.BOX_Z] = 0.19;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    // I have to flip the y values because that's what the simulation needs to reflect reality
    box_pose.position.x = -0.53; // 0.2
    box_pose.position.y = 0.36; // 0.2
    box_pose.position.z = -0.1; // 0.25

    the_floor.primitives.push_back(primitive);
    the_floor.primitive_poses.push_back(box_pose);
    the_floor.operation = the_floor.ADD;

    return the_floor;
  }();

    // Create collision object for the robot to avoid
  auto const the_post = [frame_id =
                                  move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject the_post;
    the_post.header.frame_id = frame_id;
    the_post.id = "floor";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.6;
    primitive.dimensions[primitive.BOX_Y] = 0.6;
    primitive.dimensions[primitive.BOX_Z] = 2.0;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    // I have to flip the y values because that's what the simulation needs to reflect reality
    box_pose.position.x = -1.0;
    box_pose.position.y = 1.3;
    box_pose.position.z = 0.5;

    the_post.primitives.push_back(primitive);
    the_post.primitive_poses.push_back(box_pose);
    the_post.operation = the_post.ADD;

    return the_post;
  }();


  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(the_floor);
  planning_scene_interface.applyCollisionObject(the_post);

  rclcpp::sleep_for(std::chrono::seconds(1));

  move_group_interface.setNumPlanningAttempts(50); // like why not. It's taking usually like 0.05 sec per plan
  // previously it was using the default of 10 and failing to find a solution
  move_group_interface.setPlanningTime(10.0);

  // move_group_interface.setEndEffector()

  // SECTION FOR CARTESIAN PATHS:
  // funny story: the cartesia planner just dies if it goes through the origin LOL

  // 1. Define your waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose); // The goal position

  // 2. Set parameters for the Cartesian calculation
  double eef_step = 0.01;      // Resolution: 1cm interpolation steps
  // this number might be bad and messing me up. The cartesian path planner fails a lot and this could be why?
  // regardless we have the OMPL planner, so it's fine.

  // 3. Compute the path
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_interface.computeCartesianPath(
      waypoints, 
      eef_step, 
      trajectory
  );

  // 4. Execute the plan
  if (fraction >= 1.0) {
    move_group_interface.execute(trajectory);
  } else {
    RCLCPP_WARN(node->get_logger(), "Cartesian path planning failed. Attempting non-cartesian planning.");
    
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // 4. Execute the plan
    if (success) {
      move_group_interface.execute(my_plan);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    }
  }

  // comment out the below section for not cartesian



  rclcpp::shutdown();
  return 0;
}

// #include <memory>

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.hpp>

// int main(int argc, char * argv[])
// {
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto const node = std::make_shared<rclcpp::Node>(
//     "hello_moveit",
//     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//   );

//   // Create a ROS logger
//   auto const logger = rclcpp::get_logger("hello_moveit");

//   // Create the MoveIt MoveGroup Interface
//   using moveit::planning_interface::MoveGroupInterface;
//   auto move_group_interface = MoveGroupInterface(node, "manipulator");

//   // Set a target Pose
//   auto const target_pose = []{
//     geometry_msgs::msg::Pose msg;
//     msg.orientation.w = 1.0;
//     msg.position.x = 0.28;
//     msg.position.y = -0.2;
//     msg.position.z = 0.5;
//     return msg;
//   }();
//   move_group_interface.setPoseTarget(target_pose);

//   // Create a plan to that target pose
//   auto const [success, plan] = [&move_group_interface]{
//     moveit::planning_interface::MoveGroupInterface::Plan msg;
//     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//     return std::make_pair(ok, msg);
//   }();

//   // Execute the plan
//   if(success) {
//     move_group_interface.execute(plan);
//   } else {
//     RCLCPP_ERROR(logger, "Planning failed!");
//   }

//   // Shutdown ROS
//   rclcpp::shutdown();
//   return 0;
// }