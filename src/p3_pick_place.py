
import subprocess
import argparse
import ast
import math


def main():

    # if no start position that's really bad -> fail
    # if no gripper width taht's also bad -> fail
    # the only optional are the yaw pitch roll

    parser = argparse.ArgumentParser(description="The Parasol Pick Place pick and place script")
    parser.add_argument("pick_pos_ori", type=str)
    parser.add_argument("gripper_width", type=float)
    parser.add_argument("place_pos_ori", type=str)
    args = parser.parse_args()
    
    # object_pos_ori: tuple = sys.argv[1]
    # print(object_pos_ori)
    # print(object_pos_ori[1])

    pick_pos_ori = ast.literal_eval(args.pick_pos_ori)
    gripper_width = args.gripper_width
    place_pos_ori = ast.literal_eval(args.place_pos_ori)

    if (not check_valid_goal(pick_pos_ori)):
        print("The pick position and orientation tuple has an element that is not numeric. Aborting")
        return 1
    if (not check_valid_goal(place_pos_ori)):
        print("The place position and orientation tuple has an element that is not numeric. Aborting")
        return 1
    

    if (len(pick_pos_ori) != 6):
        print("the pick position and orientation tuple does not have 6 arguments. Got this: " + str(pick_pos_ori))
        print("Aborting.")
        return 1

    if (len(place_pos_ori) == 3):
        # remake place_pos_ori to have 6 arguments
        naive_yaw = math.atan2(place_pos_ori[1], place_pos_ori[0])
        # we will use the pitch and roll that the object was picked up with, and the yaw based on the overall direction the robot points
        # note that this is VERY naive. But the motion plannner / p3_move_arm node should not do anything unsafe
        # this is purely to help find *a* valid goal configuration when the place orientation is not known. 
        place_pos_ori = (place_pos_ori[0], place_pos_ori[1], place_pos_ori[2], naive_yaw, pick_pos_ori[4], pick_pos_ori[5])
        # note that this if statement is currently 100% untested
    elif (len(place_pos_ori) != 6):
        print("the place position and orientation tuple does not have 6 arguments. Got this: " + str(place_pos_ori))
        print("Aborting.")
        return 1

    print(pick_pos_ori)
    print(gripper_width)
    print(place_pos_ori)


    # Start executing the motions
    # we will define the gripper width that we should have on approach as the grasp width
    # and then plus some constatn

    APPROACH_GRASP_DECREASE = 0.15 # this is really the increase in how open it is
    approach_grasp_width = max(gripper_width - APPROACH_GRASP_DECREASE, 0.01)

    # subprocess.run() is blocking, so each of these will execute sequentially. 

    # set the gripper to the approach width
    if (not gripper_action(approach_grasp_width, "first gripper approach width")):
        return 1

    # move to the pick position
    if (not arm_action(pick_pos_ori, "pick pre-grasp manuever", False)):
        return 1
    

    # move to the pick tcp position 
    if (not arm_action(pick_pos_ori, "pick tcp manuever", True)):
        return 1

    # set gripper to gripper width
    if (not gripper_action(gripper_width, "grasp object")):
        return 1
    
    # move back to the pick pre-grasp
    if (not arm_action(pick_pos_ori, "post-pick back to pre-grasp manuever", False)):
        return 1

    # move to the place pre-grasp position
    if (not arm_action(place_pos_ori, "place pre-grasp manuever", False)):
        return 1
    
    # move to the place tcp pos
    if (not arm_action(place_pos_ori, "place tcp manuever", True)):
        return 1
    
    # set gripper to the approach width
        # we do not want to shove it all the way open in case we are holding a cup or something and we would cause
        # a collision by doing that. 
    if (not gripper_action(approach_grasp_width, "release object")):
        return 1

    # go back to the place pre-grasp position
    if (not arm_action(place_pos_ori, "post-place back to pre-grasp manuever", False)):
        return 1


    # done!
    return 0

    # print(args.pick_pos_ori)

    # my_tuple = (1, 2.0, 4.57)
    # print(my_tuple[2])

    # my_tuple = ast.literal_eval(args.start_pos)
    # print(my_tuple)
    # print(type(my_tuple[2]))

    # result = subprocess.run([sys.executable, "test_function.py"])
    # print("Return Code:", result.returncode)
    # print("Standard Output:", result.stdout)
    # print("Standard Error:", result.stderr)


    # result = subprocess.run(["source", "../install/setup.bash"]) # we can't source. I don't fully understand why but
    # # the two second version is that subprocess.run can't modify the environment.  
    # result = subprocess.run(["ls", "-l"])
    # result = subprocess.run(["ros2", "run", "parasol_pick_place", "p3_move_gripper", "close"])
    # if (result.returncode):
    #     print("Sad day")


    # # beautiful
    # print("Return Code:", result.returncode)
    # print("Standard Output:", result.stdout)
    # print("Standard Error:", result.stderr)

    return 0


def gripper_action(desired_width, description):
    result_approach_width = subprocess.run(["ros2", "run", "parasol_pick_place", "p3_move_gripper", str(desired_width)])
    if (result_approach_width.returncode):
        print_failed_plan(description, desired_width)
        return False
    return True

def arm_action(goal_pos_ori, description, use_tcp: bool):
    parameter_list = ["ros2", "run", "parasol_pick_place", "p3_move_arm"]
    for i in range(len(goal_pos_ori)):
        parameter_list.append(str(goal_pos_ori[i]))
    
    if (use_tcp):
        parameter_list.append("tcp")

    print("Running with parameter list ", str(parameter_list))

    result = subprocess.run(parameter_list) # this will actually move the robot
    print(result.returncode)
    if (result.returncode):
        # bad. abort.
        print_failed_plan(description, goal_pos_ori)
        return False
    return True


def print_failed_plan(description, goal):
    print("Failed to plan or execute the " + description + " action to " + str(goal) + "!")


# loop through the elements and make sure each one is a float or an int
def check_valid_goal(goal):
    for val in goal:
        if ((type(val) != type(1.0)) and (type(val) != type(1))):
            return False
    return True


# def move_pre_tcp(goal_pos):

#     parameter_list = ["ros2", "run", "parasol_pick_place", "p3_move_arm"]
#     for i in range(len(goal_pos)):
#         parameter_list.append(str(goal_pos[i]))

#     result_pre = subprocess.run(parameter_list) # this will actually move the robot
#     if (result_pre.returncode):
#         # bad. abort.
#         print_failed_plan("pre-grasp manuever", goal_pos)
#         return False

#     parameter_list.append("tcp")
#     result_tcp = subprocess.run(parameter_list) # as will this
#     if (result_tcp.returncode):
#         # bad. abort.
#         print_failed_plan("tcp manuever", goal_pos)
#         return False
    
#     return True







if (__name__ == "__main__"):
    main()

