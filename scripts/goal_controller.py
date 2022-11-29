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
            curr_config = [rospy.wait_for_message('/robo_0/joint_0/angle_state', JointState, timeout=5).position,
                           rospy.wait_for_message('/robo_0/joint_1/angle_state', JointState, timeout=5).position,
                           rospy.wait_for_message('/robo_0/joint_2/angle_state', JointState, timeout=5).position]
        else:
            curr_config = [np.array([0.0,0.0])]*3
        
        self.robot_kine = kinb.Kinematics()
        self.robot_kine.update_fk(*curr_config)
        self.curr_state = self.robot_kine.get_relative_transform("JOINT0_BOTTOM", "JOINT2_TOP")[:3, -1]
        # self.curr_state = [0.0, 0.0, 0.0]

        # [lj_hor, lj_vert, rj_hor, rj_vert, l_trig, r_trig]
        self.joy_cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # commands coming directly controller [lj_hor, lj_vert, rj_hor, rj_vert]

        self.goal_pub = rospy.Publisher("/goal_position", Point, queue_size=10)

        self.curr_state_sub = rospy.Subscriber("/visualization_marker", Marker, self.curr_state_cb, queue_size=5)
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

    def curr_state_cb(self, msg):
        if msg.id == 0:
            self.curr_state = [msg.pose.position.x, 
                               msg.pose.position.y, 
                               msg.pose.position.z]

    def joy_cmd_cb(self, msg):
        # [lj_hor, lj_vert, rj_hor, rj_vert, l_trig, r_trig]
        self.joy_cmd = [-msg.axes[0],
                         msg.axes[1],
                         msg.axes[3],
                         msg.axes[4],
                         0 if msg.axes[2] == 1 else msg.axes[2], # left/right triggers start at 1 and go to -1
                         0 if msg.axes[5] == 1 else msg.axes[5]]
 
    def xbox_to_pose(self, state, cmd):
        """
        This functions converts current state and joystick commands to a new goal position
        """
        cartesian_speed = 0.01 # Speed of change in position
        # This controls x, y and z pose
        goal = [state[0] + cartesian_speed*cmd[0], # x/y position is controlled by left joystick
                state[1] + cartesian_speed*cmd[1],
                state[2] + 0.5*cartesian_speed*((cmd[4] - cmd[5]))] # z position is controlled by left/right triggers 

        self.curr_state = self.check_workspace(goal)

    def check_workspace(self, goal):
        # make sure that the goal position is in the workspace. If it's not, move it into the workspace
        goal[0] = goal[0] if np.abs(goal[0]) < 0.9 else np.sign(goal[0])*0.9
        goal[1] = goal[1] if np.abs(goal[1]) < 0.9 else np.sign(goal[1])*0.9
        if goal[2] < 0.1:
            goal[2] = 0.1
        elif goal[2] > 0.9:
            goal[2] = 0.9
        return goal


if __name__ == '__main__':
    c = GoalController()
    c.run()
