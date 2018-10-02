#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from baxter_interface import gripper as robot_gripper
import numpy as np
from numpy import linalg

def main():
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')

    gripper = robot_gripper.Gripper('left')

    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    while not rospy.is_shutdown():
        
        #Construct the request
        request1 = GetPositionIKRequest()

        request1.ik_request.group_name = "left_arm"
        request1.ik_request.ik_link_name = "left_gripper"
        request1.ik_request.attempts = 20
        request1.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector HERE
        request1.ik_request.pose_stamped.pose.position.x = .703
        request1.ik_request.pose_stamped.pose.position.y = .505
        request1.ik_request.pose_stamped.pose.position.z = -.388  
        request1.ik_request.pose_stamped.pose.orientation.x = 0.0
        request1.ik_request.pose_stamped.pose.orientation.y = 1.0
        request1.ik_request.pose_stamped.pose.orientation.z = 0.0
        request1.ik_request.pose_stamped.pose.orientation.w = 0.0

        #Request 2
        request2 = GetPositionIKRequest()

        request2.ik_request.group_name = "left_arm"
        request2.ik_request.ik_link_name = "left_gripper"
        request2.ik_request.attempts = 20
        request2.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector HERE
        request2.ik_request.pose_stamped.pose.position.x = .777
        request2.ik_request.pose_stamped.pose.position.y = .280
        request2.ik_request.pose_stamped.pose.position.z = -.372   
        request2.ik_request.pose_stamped.pose.orientation.x = 0.0
        request2.ik_request.pose_stamped.pose.orientation.y = 1.0
        request2.ik_request.pose_stamped.pose.orientation.z = 0.0
        request2.ik_request.pose_stamped.pose.orientation.w = 0.0        
        
        try:
            #Send the request to the service
            response = compute_ik(request1)
            
            #Print the response HERE
            print(response)
            group = MoveGroupCommander("left_arm")

            # Setting position and orientation target
            group.set_pose_target(request1.ik_request.pose_stamped)

            # Plan IK and execute
            group.go()
            
            #Calibration
            print('Calibrating...')
            gripper.calibrate()
            rospy.sleep(2.0)

            #Close the gripper
            print('Closing...')
            gripper.close()
            rospy.sleep(1.0)

            response2 = compute_ik(request2)
            
            #Print the response HERE
            print(response2)
            group = MoveGroupCommander("left_arm")

            # Setting position and orientation target
            group.set_pose_target(request2.ik_request.pose_stamped)

            # Plan IK and execute
            group.go()

            #Open the gripper
            print('Opening...')
            gripper.open()
            rospy.sleep(1.0)
            print('Done!')


        except rospy.ServiceException, e:
            print "Service call failed: %s"%e



#Python's syntax for a main() method
if __name__ == '__main__':
    main()

