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

  // 1. Get the current pose of the end-effector (usually "tool0")
  auto live_pose = move_group_interface.getCurrentPose(move_group_interface.getEndEffectorLink());


  // I'm getting the current position in x y z and the current orientation in yaw pitch roll
  // MoveIt operates using quaternions because they avoid issues like gimbal lock. So I have to convert
  // from the quaternion to yaw pitch and roll
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

  const char* do_nothing_str = "x"; // if the user passes in this string, the current position of the robot
  // for the substituted argument will be used.

  // 1. Take the user input in degrees and store it as radians
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

  target_pose.position.x = (!strcmp(do_nothing_str, argv[1])) ? live_pose.pose.position.x : std::stod(argv[1]);
  target_pose.position.y = (!strcmp(do_nothing_str, argv[2])) ? live_pose.pose.position.y : std::stod(argv[2]);
  target_pose.position.z = (!strcmp(do_nothing_str, argv[3])) ? live_pose.pose.position.z : std::stod(argv[3]);

  
  RCLCPP_INFO(node->get_logger(), "Goal Quaternion: x=%f, y=%f, z=%f, w=%f",
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w);          


  move_group_interface.setPoseTarget(target_pose);

  // If you wanted to send a joint-space command, this is how you could do it:
  // I think (emphasis on think) this position is very similar to the upright home position
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

  
  // Create a collision object for the robot to avoid
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
    primitive.dimensions[primitive.BOX_Z] = 0.19; // this is the diameter

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    // I have to flip the y values because that's what the simulation needs to reflect reality
    box_pose.position.x = -0.53; // these x and y positions create the floor with the right orientation relative to the UR5e in 3307
    box_pose.position.y = 0.36; 
    box_pose.position.z = -0.1; 

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
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -1.0;
    box_pose.position.y = 1.3;
    box_pose.position.z = 0.5;

    the_post.primitives.push_back(primitive);
    the_post.primitive_poses.push_back(box_pose);
    the_post.operation = the_post.ADD;

    return the_post;
  }();


  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Create the planning scene (above) and add the collision objects to it (below)
  planning_scene_interface.applyCollisionObject(the_floor);
  planning_scene_interface.applyCollisionObject(the_post);
  // even though planning_scene_interface isn't used anywhere else, MoveIt will still account for it, and its obstacles will persist
  // even after this ROS node ends.

  rclcpp::sleep_for(std::chrono::seconds(1));

  move_group_interface.setNumPlanningAttempts(50); // This could very likely be a much higher value
  move_group_interface.setPlanningTime(5.0);


  // SECTION FOR CARTESIAN PATHS:
  // funny story: the cartesia planner just dies if it goes through the origin LOL

  // Cartesian Paths:
  // We first attempt to create a cartesian path for a smoother motion. If this fails, we will try the full motion planner

  // 1. Define your waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose); // The goal position

  // 2. Set parameters for the Cartesian calculation
  // Resolution: 1cm interpolation steps
  double eef_step = 0.01;      

  // 3. Compute the path
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_interface.computeCartesianPath(
      waypoints, 
      eef_step, 
      trajectory
  );

  // 4. Execute the plan
  if (fraction >= 1.0) {
    // we successfully found a cartesian path
    move_group_interface.execute(trajectory);
  } else {
    // we could not find a cartesian path. Instead, we will now use the actual planner
    RCLCPP_WARN(node->get_logger(), "Cartesian path planning failed. Attempting non-cartesian planning.");
    
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // 4. Execute the plan
    if (success) {
      move_group_interface.execute(my_plan);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    }
  }

  rclcpp::shutdown();
  return 0;
}