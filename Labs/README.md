# Labs FAQ and remarks

## Lab1 and Lab2
These two labs contain a simple introduction to ROS, including ROS publisher, subscriber, node and some stuff about python. It should be quite easy if you follow the ROS official tutorials.

One small tip is presented for Part 4 in Lab 2. In order to create multiple turtles in the simulation. I would recommend to use ` rosservice`, such as `rosservice call spawn 1 1 0 turtle`, where three double represent the x, y postion and the rotational angle, respectively.

## Lab3
Please refer to quaternion representation and transformation matrix involving twist representation, which is strongly recommended.

If you have difficulty with Part 1.4, I would recommend you to use `rosrun tf view_frames`, which generates a list of frames in the terminal and summary them up in a pdf file. As you might noticed, the system has more than thirty frames. It's fine to choose any two of them to finish part 1.4.

## Lab3 Baxter
For part 2 of lab 3, we need to use function `set_joint_positions()` to enable the movement to the desired position. Initially we move the robot to a desired position with following codes, 
```
bindings = {
    'left_s0': float(raw_input("Type the angle")),
    'left_s1': float(raw_input("Type the angle")),
    'left_e0': float(raw_input("Type the angle")),
    'left_e1': float(raw_input("Type the angle")),
    'left_w0': float(raw_input("Type the angle")),
    'left_w1': float(raw_input("Type the angle")),
    'left_w2': float(raw_input("Type the angle"))
    }
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
    	try:
    		left.set_joint_positions(bindings)    
    	except KeyboardInterrupe:
    		print("escape")
```
With this kind of code, the program won't be stopped automatically when the desired rotational angles are satisfied. Therefore, we need add an uncertainty for the convergence of angle errors. A sample code can be written as follow,
```
our_theta={}
our_theta['right_s0']=np.float32(raw_input())
our_theta['right_s1']=np.float32(raw_input())
our_theta['right_e0']=np.float32(raw_input())
our_theta['right_e1']=np.float32(raw_input())
our_theta['right_w0']=np.float32(raw_input())
our_theta['right_w1']=np.float32(raw_input())
our_theta['right_w2']=np.float32(raw_input())

while( right.joint_angle(lj[0])-our_theta['right_s0']<0.1 or
   right.joint_angle(lj[1])-our_theta['right_s1']<0.1 or
   right.joint_angle(lj[2])-our_theta['right_e0']<0.1 or
   right.joint_angle(lj[3])-our_theta['right_e1']<0.1 or
   right.joint_angle(lj[4])-our_theta['right_w0']<0.1 or
   right.joint_angle(lj[5])-our_theta['right_w1']<0.1 or
   right.joint_angle(lj[6])-our_theta['right_w2']<0.1 ):
 right.set_joint_positions(our_theta)
```


## Lab4
This lab talks about an introductive calibration of camera, including an implementation of Homography matrix and numerical calculations of mapping from pixel coordinates to world coordinates. By following the intructions, this lab work is easily to be finished. 
