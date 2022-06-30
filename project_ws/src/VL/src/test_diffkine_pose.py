#!/usr/bin/env python
#
 
from __future__ import print_function
import rospy
from sensor_msgs.msg import JointState
 
from pyquaternion import Quaternion
 
from markers import *
from functions import *
 
 
# Initialize the node
rospy.init_node("testKineControlPose")
print('starting motion ... ')
# Publisher: publish to the joint_states topic
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
# Markers for the current and desired positions
bmarker_current  = FrameMarker()
bmarker_desired = FrameMarker(0.5)
 
# Joint names
jnames = ['waist_q1', 'shoulder_q2', 'revolution_q3','elbow_q4', 'slider_q5', 'wrist_q6']

# Desired pose
ang = pi/3
# Rd = np.array([[0,1,0],[1,0,0],[0,0,-1]])
Rd = np.array([[0,1,0],[1,0,0],[0,0,-1]])
tempq = Quaternion(matrix=Rd)
qd = np.array([tempq.w, tempq.x, tempq.y, tempq.z])
 
qd = rot2quat(Rd)
 
# Find an xd that the robot can reach
xd = np.array([0, 0, 0.9, qd[0], qd[1], qd[2], qd[3]])
#xd  = np.array([0.5, 0.5, 0.6, np.cos(ang/2.0), 0, 0, np.sin(ang/2.0)])
# Initial configuration
q0  = np.array([0, 0, 0, 0, 0, 0])
 
# Resulting initial pose (end effector with respect to the base link)
T = fkine(q0)
x0 = TF2xyzquat(T)
 
epsilon = 0.000001
k = 0.5
count = 0
 
# Markers for the current and the desired pose
bmarker_current.setPose(x0)
bmarker_desired.setPose(xd)
 
# Instance of the JointState message
jstate = JointState()
# Values of the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q0
 
# Frequency (in Hz) and control period 
freq = 200
dt = 1.0/freq
rate = rospy.Rate(freq)
 
# Initial joint configuration
q = copy(q0)
x = copy(x0)
quat = x[3:7]
# Initialize the derror vector (derivative of the error)
derror = np.zeros(7)
e = np.zeros(7)
 
quat_wd = qd[0]
quat_ed = qd[1:4]
 
# Main loop
#for i in range(1):
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Kinematic control law for the pose (complete here)
    # --------------------------------------------------
 
    J = jacobian_pose(q, 0.0001)
 
    quat_w = x[3]
    quat_e = x[4:7]
 
    e[0:3] = x[0:3] - xd[0:3]
 
    # e[3] = quat_wd*quat_w + quat_ed.T.dot(quat_e)
 
    e[3] = quat_wd*quat_w + (quat_ed.T).dot(quat_e) - 1
    e[4:7] = -quat_wd*quat_e + quat_w*quat_ed - np.cross(quat_ed,quat_e)
 
    if(np.linalg.norm(e)-1 < epsilon):
        print('Desired pose reached')
        print('Quaternion:', quat)
        print('Rotation Matrix', Q_R(quat))
        print('TH final', fkine(q))
        print('TH inicial', fkine(q0))
        break
 
    """
    rankJ = matrix_rank(jacobian_pose(q))
    if (rankJ == 7):
        dq = np.linalg.pinv(J).dot(de)
    elif (rankJ < 7):
        dampedinvJ = J.T.dot(np.linalg.inv(J.dot(J.T) + 4*np.ones((7, 7))))
        dq = dampedinvJ.dot(de)
    """
    derror = -k*e
    J_inv = np.linalg.inv(J.T.dot(J)).dot(J.T)
    dq = J_inv.dot(derror)
    q = q + dt*dq
    
    count = count + 1
 
    print(np.linalg.norm(e))
 
    if(count > 10000):
        print('Max number of iterations reached')
        break
 
    # Current configuration trnaformation to current position
    T = fkine(q)
    T_co = np.array([[0,1,0],[1,0,0],[0,0,-1]])
    x = TF2xyzquat(T)
    # Publish the message
    jstate.position = q
    pub.publish(jstate)
    bmarker_desired.setPose(xd)
    bmarker_current.setPose(x)
    # Wait for the next iteration
    rate.sleep()
