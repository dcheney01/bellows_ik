#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Joy
import numpy as np
from sensor_msgs.msg import JointState
from arm_kinematics import kinematics as kinb


# Author: Daniel Cheney
"""
This file allows for control of cartesian coordinates of a point
"""

class GoalController():
    def __init__(self):
        pass

    def setup(self, control_rate=200.0):

        if rospy.get_param("/is_hardware"):
            # Get the initial configuration of the joints
            curr_config = [rospy.wait_for_message('/robo_0/joint_0/angle_state', JointState, timeout=5).position,
                           rospy.wait_for_message('/robo_0/joint_1/angle_state', JointState, timeout=5).position,
                           rospy.wait_for_message('/robo_0/joint_2/angle_state', JointState, timeout=5).position]
            self.robot_kine = kinb.Kinematics()
            self.robot_kine.update_fk(*curr_config)
            self.curr_state = self.robot_kine.get_relative_transform("JOINT0_BOTTOM", "JOINT2_TOP")[:3, -1]
        else:
            msg = rospy.wait_for_message('/current_position', Point, timeout=5)
            self.curr_state = [msg.x, msg.y, msg.z]
        
        # [lj_hor, lj_vert, rj_hor, rj_vert, l_trig, r_trig]
        self.joy_cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # commands coming directly controller [lj_hor, lj_vert, rj_hor, rj_vert]

        self.goal_pub = rospy.Publisher("/goal_position", Point, queue_size=10)

        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cmd_cb)

        self._control_rate = rospy.Rate(control_rate)

    def run(self):
        """Runs the controller node"""
        rospy.init_node('goal_controller')

        self.setup()

        while not rospy.is_shutdown():
            self.xbox_to_pose(self.curr_state, self.joy_cmd)
            goal_msg = Point(x=self.curr_state[0], 
                             y=self.curr_state[1], 
                             z=self.curr_state[2])
            self.goal_pub.publish(goal_msg)
            self._control_rate.sleep()

    def joy_cmd_cb(self, msg):
        # [lj_hor, lj_vert, rj_hor, rj_vert, l_trig, r_trig]
        self.joy_cmd = [ msg.axes[0],
                         msg.axes[1],
                         msg.axes[3],
                         msg.axes[4],
                         0 if msg.axes[2] == 1 else msg.axes[2], # left/right triggers start at 1 and go to -1
                         0 if msg.axes[5] == 1 else msg.axes[5]]
 
    def xbox_to_pose(self, state, cmd):
        """
        This functions converts current state and joystick commands to a new goal position
        """
        cartesian_speed = 0.001 # Speed of change in position
        # This controls x, y and z pose
        goal = [state[0] + cartesian_speed*cmd[1], # x position is controlled by left joystick horizontally
                state[1] + cartesian_speed*(cmd[0] + cmd[2]), # y position is controlled by left joystick vertically + right joystick horizontally
                state[2] + 0.5*cartesian_speed*cmd[3]] # z position is controlled by left joystick vertically 

        self.curr_state = self.check_workspace(goal)

    def check_workspace(self, goal):
        # make sure that the goal position is in the workspace. If it's not, move it back into the workspace
        # X limits
        goal[0] = goal[0] if np.abs(goal[0]) < 1 else np.sign(goal[0])

        # Y limits
        goal[1] = goal[1] if np.abs(goal[1]) < 1 else np.sign(goal[1])

        # Z limits
        if goal[2] < 0.1:
            goal[2] = 0.1
        elif goal[2] > 1:
            goal[2] = 1
        return goal


if __name__ == '__main__':
    c = GoalController()
    c.run()
