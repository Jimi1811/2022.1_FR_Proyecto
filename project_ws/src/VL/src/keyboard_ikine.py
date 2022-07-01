#!/usr/bin/env python
 
import rospy
 
# Publisher
from sensor_msgs.msg import JointState
from markers import *
from functions import *
 
# Subscriber
from std_msgs.msg import String
global press_key
press_key = "0"
def callback(msg):
    global press_key
    press_key = msg.data
 
# Main
if __name__ == '__main__':
    # Nodo
    rospy.init_node("keyboard_ikine")
    # Publisher
    pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
    bmarker = BallMarker(color['GREEN'])
    # Subscriber
    rospy.Subscriber("/keys", String, callback)
 
    # -------------- Pelotita y robot --------------
    # Joint names
    jnames = ['waist_q1', 'shoulder_q2', 'revolution_q3','elbow_q4', 'slider_q5', 'wrist_q6']
    # Joint Configuration
    q0 = [0, 0, 0, 0, 0, 0]  # CI  
    q = q0
 
    # End effector with respect to the base
    T = fkine(q0)
    print( np.round(T, 3) )
    bmarker.position(T)
 
    xdes = T[0:3,3]
 
    deltax = [0, 0, 0]
    l_max = 0.954 + 0.2
 
    # Object (message) whose type is JointState
    jstate = JointState()
    # Set values to the message
    jstate.header.stamp = rospy.Time.now()
    jstate.name = jnames
    # Add the head joint value (with value 0) to the joints
    jstate.position = q0
 
 
    # Loop rate (in Hz)
    rate = rospy.Rate(10)
 
    # Continuous execution loop
    while not rospy.is_shutdown():
 
        # Muestra tecla
 
        # x+
        if (press_key == 'w'):
            print 'Input:', press_key
            deltax[0] = 0.02
                
        # x-
        elif (press_key == 's'):
            print 'Input:', press_key
            deltax[0] = -0.02
 
        # y+
        elif (press_key == 'a'):
            print 'Input:', press_key
            deltax[1] = 0.02
 
        # y-
        elif (press_key == 'd'):
            print 'Input:', press_key
            deltax[1] = -0.02
 
        # z+
        elif (press_key == 'r'):
            print 'Input:', press_key
            deltax[2] = 0.02
 
        # z-
        elif (press_key == 'f'):
            print 'Input:', press_key
            deltax[2] = -0.02
 
 
        if (np.linalg.norm(xdes + deltax) < l_max*0.95):
            xdes = xdes + deltax
        else:
            print('Largo maximo alcanzado')
 
        if (deltax != [0, 0, 0]):
            q,ee = ikine(xdes,q0)
            q0 = q
            deltax = [0, 0, 0]
 
        # Current time (needed for ROS)
        jstate.header.stamp = rospy.Time.now()
        # Add the head joint value (with value 0) to the joints
        jstate.position = q
        # Publish the message
        pub.publish(jstate)
        bmarker.position(T)
        bmarker.publish()
 
        # Wait for the next iteration
        rate.sleep()
