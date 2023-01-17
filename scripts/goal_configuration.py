#!/usr/bin/env python

import rospy
from arm_kinematics import kinematics as kinb # RaD Lab Package
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from copy import deepcopy

"""
This file subscribes to /goal_position and /current_position to publish /goal_configuration
"""

class GoalConfigurationIK():
    def __init__(self):
        pass

    def setup(self, control_rate=200.0):
        self.isHardware = rospy.get_param("/is_hardware")

        self.goal_position = np.array([0.0]*3)
        self.curr_config = [np.array([0.0,0.0])]*3
        self.robot_kine = kinb.Kinematics()

        if self.isHardware:
            self.jt0_config_sub = rospy.Subscriber("/robo_0/joint_0/angle_state", JointState, self.jt0_config_sub_cb, queue_size=5)
            self.jt1_config_sub = rospy.Subscriber("/robo_0/joint_1/angle_state", JointState, self.jt1_config_sub_cb, queue_size=5)
            self.jt2_config_sub = rospy.Subscriber("/robo_0/joint_2/angle_state", JointState, self.jt2_config_sub_cb, queue_size=5)

            self.joint0_cmd_pub = rospy.Publisher("/robo_0/joint_0/joint_cmd", JointState, queue_size=5)
            self.joint1_cmd_pub = rospy.Publisher("/robo_0/joint_1/joint_cmd", JointState, queue_size=5)
            self.joint2_cmd_pub = rospy.Publisher("/robo_0/joint_2/joint_cmd", JointState, queue_size=5)

        self.goal_position_sub = rospy.Subscriber("/goal_position", Point, self.goal_position_cb, queue_size=5)
        self.current_position_pub = rospy.Publisher("/current_position", Point, queue_size=5)

        self._control_rate = rospy.Rate(control_rate)

    def run(self):
        """Runs the ik node"""
        rospy.init_node('goal_configuration')

        self.setup()

        while not rospy.is_shutdown():
            # Only perform ik if we have a certain amount of error in position
            if np.linalg.norm(self.robot_kine.get_relative_transform("JOINT0_BOTTOM", "JOINT2_TOP")[:3, -1] - \
                        self.goal_position) > 0.01:
                goal_config = self.ik(self.curr_config, self.goal_position) # qf from ik
                if self.isHardware:
                    self.joint0_cmd_pub.publish(JointState(position=goal_config[0:2]))
                    self.joint1_cmd_pub.publish(JointState(position=goal_config[2:4]))
                    self.joint2_cmd_pub.publish(JointState(position=goal_config[4:6]))

            curr = self.robot_kine.get_relative_transform("JOINT0_BOTTOM", "JOINT2_TOP")[:3, -1]
            self.current_position_pub.publish(Point(x=curr[0],
                                                    y=curr[1],
                                                    z=curr[2]))
            self._control_rate.sleep()

    def goal_position_cb(self, msg):
        self.goal_position = np.array([msg.x,
                                       msg.y,
                                       msg.z])

    def jt0_config_sub_cb(self, msg):
        # self.curr_config[0] = np.array([msg.position]) # u and v from msg
        self.curr_config[0] = msg.position # u and v from msg
        self.robot_kine.update_fk(*self.curr_config)


    def jt1_config_sub_cb(self, msg):
        # self.curr_config[1] = np.array([msg.position]) # u and v from msg
        self.curr_config[1] = msg.position # u and v from msg
        self.robot_kine.update_fk(*self.curr_config)


    def jt2_config_sub_cb(self, msg):
        # self.curr_config[2] = np.array([msg.position]) # u and v from msg
        self.curr_config[2] = msg.position # u and v from msg
        self.robot_kine.update_fk(*self.curr_config)


    def ik(self, curr_pose, goal_position):
        # The if statement is because robot.ik_position updates the model that this class has of the bellows arm
            # We only want to update the current position from the VR trackers for hardware so we need to make a copy
        if self.isHardware:
            temp_robot = deepcopy(self.robot_kine)
            qf, error_f, iter, reached_max_iter, status_msg = temp_robot.ik_position(np.array(goal_position), 
                                                                                curr_pose[0],
                                                                                curr_pose[1],
                                                                                curr_pose[2],
                                                                                method='J_T', 
                                                                                K = np.eye(3),
                                                                                max_iter = 500)
        else: # running on simulation so we can update our model directly
            # self.robot_kine.update_fk(*curr_pose)
            qf, error_f, iter, reached_max_iter, status_msg = self.robot_kine.ik_position(np.array(goal_position), 
                                                                                        curr_pose[0],
                                                                                        curr_pose[1],
                                                                                        curr_pose[2],
                                                                                        method='J_T', 
                                                                                        K = np.eye(3),
                                                                                        max_iter = 500)

        # print(qf, error_f, iter, reached_max_iter, status_msg)
        return qf
        
    
if __name__ == '__main__':
    ik = GoalConfigurationIK()
    ik.run()