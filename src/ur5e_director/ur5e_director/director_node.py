# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class UR5e_Director(Node):

    def __init__(self):
        super().__init__('ur5e_director')
        self.publisher_ = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)

    def move_arm_script(self):
        msg = JointTrajectory()
        msg.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        # Create a trajectory point
        point = JointTrajectoryPoint()
        
        # Define positions in radians (this is a slight "L" shape)
        point.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        
        # Tell the robot to reach this position in 2 seconds
        point.time_from_start = Duration(sec=2, nanosec=0)

        msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info("Sending the command to the UR5e...")


def main(args=None):
    rclpy.init(args=args)

    ur5e_director = UR5e_Director()

    import time
    time.sleep(1.0)

    ur5e_director.move_arm_script()

    # Spin once to ensure the middleware processes the outgoing message
    rclpy.spin_once(ur5e_director, timeout_sec = 1.0)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ur5e_director.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
