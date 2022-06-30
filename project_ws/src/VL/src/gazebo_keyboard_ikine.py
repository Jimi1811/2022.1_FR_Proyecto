#!/usr/bin/env python
import time
# import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from functions import *
 
# Subscriber
from std_msgs.msg import String
global press_key
press_key = "0"
def callback(msg):
    global press_key
    press_key = msg.data
 
 
if __name__ == '__main__':
    
    rospy.init_node("test1", disable_signals=True)
 
    # Subscriber
    rospy.Subscriber("/keys", String, callback)
 
    robot_client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
 
    print "Waiting for server..."
    robot_client.wait_for_server()
    print "Connected to server"
 
    joint_names = ['waist_q1', 'shoulder_q2', 'revolution_q3','elbow_q4', 'slider_q5', 'wrist_q6']
    Q0 = [0.0, 0, 0, 0, 0, 0.0]
    q = [0, 0, 0, 0, 0, 0]
 
    T = fkine(Q0)
    xdes = T[0:3,3]
 
    deltax = [0, 0, 0]
 
    l_max = 0.954 + 0.2
 
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = joint_names
 
    # Initial position
    g.trajectory.points = [ JointTrajectoryPoint(positions=Q0, velocities=[0]*6,time_from_start=rospy.Duration(2.0))]
    robot_client.send_goal(g)
    robot_client.wait_for_result()
    rospy.sleep(1)
 
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
            potato = True
            robot_client.cancel_goal()
 
            # Modification of the motion
            # x+
            if (press_key == 'w'):
                print 'Input:', press_key
                deltax[0] = 0.02
                    
            # x-
            elif (press_key == 's'):
                print 'Input:', press_key
                deltax[0] = -0.02
 
            # y+
            elif (press_key == 'd'):
                print 'Input:', press_key
                deltax[1] = 0.02
 
            # y-
            elif (press_key == 'a'):
                print 'Input:', press_key
                deltax[1] = -0.02
 
            # z+
            elif (press_key == 'e'):
                print 'Input:', press_key
                deltax[2] = 0.02
 
            # z-
            elif (press_key == 'q'):
                print 'Input:', press_key
                deltax[2] = -0.02
 
            if (np.linalg.norm(xdes + deltax) < l_max*0.95):
                xdes = xdes + deltax
            else:
                print('Largo maximo alcanzado')
                
            q = ikine(xdes, Q0)
 
            Q0 = q
 
            deltax = [0, 0, 0]
 
            g.trajectory.points = [ JointTrajectoryPoint(positions=Q0, velocities=[0]*6, time_from_start=rospy.Duration(0.1))]
            robot_client.send_goal(g)
            robot_client.wait_for_result()
 
            rate.sleep()
 
    robot_client.cancel_goal()
