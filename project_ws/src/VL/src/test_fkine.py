#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

import numpy as np

from markers import *
from functions import *

rospy.init_node("testForwardKinematics")
pub = rospy.Publisher('joint_states', JointState, queue_size=1)
bmarker = BallMarker(color['GREEN'])

# Joint names
jnames = ['waist_q1', 'shoulder_q2', 'revolution_q3','elbow_q4', 'slider_q5', 'wrist_q6']
# Joint Configuration
q = [0, 0, 0, 0, 0, 0]

# End effector with respect to the base
T = fkine(q)
print( np.round(T, 3) )
bmarker.position(T)

# Object (message) whose type is JointState
jstate = JointState()
# Set values to the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q

# Loop rate (in Hz)
rate = rospy.Rate(20)
# Continuous execution loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    # Wait for the next iteration
    rate.sleep()
