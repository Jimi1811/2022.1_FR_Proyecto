#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from markers import *
from functions import *


rospy.init_node("testInvKine")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)

bmarker      = BallMarker(color['RED'])
bmarker_des  = BallMarker(color['GREEN'])

# Joint names
jnames = ['waist_q1', 'shoulder_q2', 'revolution_q3','elbow_q4', 'slider_q5', 'wrist_q6']

# Desired position
xd = np.array([0.8, 0.1, 0])
# Initial configuration
q0 = np.array([0.0, 0, 0, 0, 0, 0])
# Inverse kinematics
q = ikine(xd, q0)

# Resulting position (end effector with respect to the base link)
T = fkine(q)
print('Obtained value:\n', np.round(T,3))

# Red marker shows the achieved position
bmarker.xyz(T[0:3,3])
# Green marker shows the desired position
bmarker_des.xyz(xd)

# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Asignar valores al mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q

# Loop rate (in Hz)
rate = rospy.Rate(100)
# Continuous execution loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    bmarker_des.publish()
    # Wait for the next iteration
    rate.sleep()
