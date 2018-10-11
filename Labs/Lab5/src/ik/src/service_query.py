#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
import kin_func_skeleton as kfs
import numpy as np
import math

def twist(q,w):     
    t = np.zeros(6)
    t[0:3] = np.cross(-w,q)
    t[3:6] = w
    return t

def main():
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    while not rospy.is_shutdown():
        x = float(raw_input('Enter x coordinate'))
        y = float(raw_input('Enter y coordinate'))
        z = float(raw_input('Enter z coordinate'))
        
        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_gripper"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z
        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            #Send the request to the service
            response = compute_ik(request)
            
            #Print the response HERE
            print(response)

            s0 = float(raw_input("s0"))
            s1 = float(raw_input("s1"))
            e0 = float(raw_input("e0"))
            e1 = float(raw_input("e1"))
            w1 = float(raw_input("w1"))
            w2 = float(raw_input("w2"))
            w3 = float(raw_input("w3"))
            theta = [s0, s1, e0, e1, w1, w2, w3]
            print('Angles list (left hand)\n')
            print(theta)
            q = np.ndarray((3,8))
            w = np.ndarray((3,7))

            q[0:3,0] = [0.0635, 0.2598, 0.1188]
            q[0:3,1] = [0.1106, 0.3116, 0.3885]
            q[0:3,2] = [0.1827, 0.3838, 0.3881]
            q[0:3,3] = [0.3682, 0.5684, 0.3181]
            q[0:3,4] = [0.4417, 0.6420, 0.3177]
            q[0:3,5] = [0.6332, 0.8337, 0.3067]
            q[0:3,6] = [0.7152, 0.9158, 0.3063]
            q[0:3,7] = [0.7957, 0.9965, 0.3058]

            w[0:3,0] = [-0.0059,  0.0113,  0.9999]
            w[0:3,1] = [-0.7077,  0.7065, -0.0122]
            w[0:3,2] = [ 0.7065,  0.7077, -0.0038]
            w[0:3,3] = [-0.7077,  0.7065, -0.0122]
            w[0:3,4] = [ 0.7065,  0.7077, -0.0038]
            w[0:3,5] = [-0.7077,  0.7065, -0.0122]
            w[0:3,6] = [ 0.7065,  0.7077, -0.0038]
            R = np.array([[0.0076, 0.0001, -1.0000],[-0.7040, 0.7102, -0.0053],[0.7102, 0.7040, 0.0055]]).T
            t = np.zeros((6,7))
            for i in range(0,7):
                t[:,i] = twist(q[0:3,i],w[0:3,i])
            trans = kfs.prod_exp(t,theta)
            twist_arm = np.ndarray((4,4))
            twist_arm[0:3,0:3] = R
            twist_arm[0:3,3] = q[0:3,7]
            twist_arm[3,3] = 1
            trans = np.dot(trans,twist_arm)
            print(trans)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':
    main()

