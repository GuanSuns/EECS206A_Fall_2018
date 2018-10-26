#!/usr/bin/env python
"""
Path Planning Script for Lab 8
Author: Valmik Prabhu
"""

import sys
import rospy
import numpy as np

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
from baxter_interface import Limb
from controller import Controller 
# from intera_interface import Limb

def main():
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")

    Kp = 0.1 * np.array([0.3, 2, 1, 1.5, 2, 2, 3]) # Stolen from 106B Students
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.5, 0.5, 0.5]) # Stolen from 106B Students
    Ki = 0.01 * np.array([1, 1, 1, 1, 1, 1, 1]) # Untuned
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9]) # Untuned
    limb = Limb("right")
    controller = Controller(Kp, Ki, Kd, Kw, limb)

    ##
    ## Add the obstacle to the planning scene here
    ##
    # obs_pose = PoseStamped()
    # obs_pose.header.frame_id = "base"
    # obs_pose.pose.position.x = 0.5
    # obs_pose.pose.position.y = 0
    # obs_pose.pose.position.z = 0
    # obs_pose.pose.orientation.x = 0
    # obs_pose.pose.orientation.y = 0
    # obs_pose.pose.orientation.z = 0
    # obs_pose.pose.orientation.w = 1
    # planner.add_box_obstacle(np.array([0.4,1.2,0.10]), 'obs', obs_pose)



    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;

    while not rospy.is_shutdown():

        while not rospy.is_shutdown():
            limb = Limb("right")
            controller = Controller(Kp, Ki, Kd, Kw, limb)
            try:
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                goal_1.pose.position.x = 0.4
                goal_1.pose.position.y = -0.3
                goal_1.pose.position.z = 0.2

                #Orientation as a quaternion
                goal_1.pose.orientation.x = 0.0
                goal_1.pose.orientation.y = -1.0
                goal_1.pose.orientation.z = 0.0
                goal_1.pose.orientation.w = 0.0

                plan = planner.plan_to_pose(goal_1, list())

                raw_input("Press <Enter> to move the right arm to goal pose 1: ")
                if not controller.execute_path(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

        while not rospy.is_shutdown():
            limb = Limb("right")
            controller = Controller(Kp, Ki, Kd, Kw, limb)
            try:
                goal_2 = PoseStamped()
                goal_2.header.frame_id = "base"

                #x, y, and z position
                goal_2.pose.position.x = 0.4
                goal_2.pose.position.y = -0.3
                goal_2.pose.position.z = 0.0

                #Orientation as a quaternion
                goal_2.pose.orientation.x = 0.0
                goal_2.pose.orientation.y = -1.0
                goal_2.pose.orientation.z = 0.0
                goal_2.pose.orientation.w = 0.0

                plan = planner.plan_to_pose(goal_2, list())

                raw_input("Press <Enter> to move the right arm to goal pose 2: ")
                if not controller.execute_path(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

        while not rospy.is_shutdown():
            limb = Limb("right")
            controller = Controller(Kp, Ki, Kd, Kw, limb)
            try:
                goal_3 = PoseStamped()
                goal_3.header.frame_id = "base"

                #x, y, and z position
                goal_3.pose.position.x = 0.6
                goal_3.pose.position.y = -0.1
                goal_3.pose.position.z = 0.1

                #Orientation as a quaternion
                goal_3.pose.orientation.x = 0.0
                goal_3.pose.orientation.y = -1.0
                goal_3.pose.orientation.z = 0.0
                goal_3.pose.orientation.w = 0.0

                plan = planner.plan_to_pose(goal_3, list())

                raw_input("Press <Enter> to move the right arm to goal pose 3: ")
                if not controller.execute_path(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
