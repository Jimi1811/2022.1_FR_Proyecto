#!/usr/bin/env python
 
import rospy
 
# Publisher
from sensor_msgs.msg import JointState
# from markers import *
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
    rospy.init_node("keyboard_diffkine")
    # Publisher
    pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
    # bmarker = BallMarker(color['GREEN'])
    # Subscriber
    rospy.Subscriber("/keys", String, callback)
 
    # -------------- Pelotita y robot --------------
    # Joint names
    jnames = ['waist_q1', 'shoulder_q2', 'revolution_q3','elbow_q4', 'slider_q5', 'wrist_q6']
    # Joint Configuration
    q0 = [0, 0, 0, 0, 0, 0]  # CI  
    q = q0
 
    epsilon = 0.0001
    k = 0.5
 
    derror = np.zeros(7)
    e = np.zeros(7)
 
    # ARREGLAR
     
    # End effector with respect to the base
    T = fkine(q0)
    print( np.round(T, 3) )
    # bmarker.position(T)
 
    xdes = T[0:3,3]
 
    qd = rot2quat(T[0:3,0:3])
 
    quat_wd = qd[0]
    quat_ed = qd[1:4]
 
    deltax = [0, 0, 0]
    l_max = 0.95 + 0.2
 
    # Object (message) whose type is JointState
    jstate = JointState()
    # Set values to the message
    jstate.header.stamp = rospy.Time.now()
    jstate.name = jnames
    # Add the head joint value (with value 0) to the joints
    jstate.position = q0
 
 
    # Loop rate (in Hz)
    freq = 200
    dt = 1.0/freq
    rate = rospy.Rate(freq)
 
    # Continuous execution loop
    while not rospy.is_shutdown():
 
        # Muestra tecla
 
        # x+
        if (press_key == 'w'):
            print('Input:', press_key)
            deltax[0] = 0.05
                
        # x-
        elif (press_key == 's'):
            print('Input:', press_key)
            deltax[0] = -0.05
 
        # y+
        elif (press_key == 'a'):
            print('Input:', press_key)
            deltax[1] = 0.05
 
        # y-
        elif (press_key == 'd'):
            print('Input:', press_key)
            deltax[1] = -0.05
 
        # z+
        elif (press_key == 'r'):
            print('Input:', press_key)
            deltax[2] = 0.05
 
        # z-
        elif (press_key == 'f'):
            print('Input:', press_key)
            deltax[2] = -0.05
 
 
        if (np.linalg.norm(xdes + deltax) < l_max*0.95):
            xdes = xdes + deltax
        else:
            print('Largo maximo alcanzado')
 
        if (deltax != [0, 0, 0]):
            
            # Aqui va el codigo del control diferencial
 
            count = 0
            xd = xdes
            x = xdes - deltax
            
            while not rospy.is_shutdown():
            
                J = jacobian_pose(q)
                quat = rot2quat(fkine(q)[0:3, 0:3])
                quat_w = quat[0]
                quat_e = quat[1:4]
 
                e[0:3] = x[0:3] - xd[0:3]
 
                # e[3] = quat_wd*quat_w + quat_ed.T.dot(quat_e)
                e[3] = quat_wd*quat_w + quat_ed.T.dot(quat_e) - 1
                e[4:7] = -quat_wd*quat_e + quat_w*quat_ed - np.cross(quat_ed,quat_e)
 
                if(np.linalg.norm(e) < epsilon):
                    print('Desired pose reached')
                    print('Quaternion:', quat)
                    print('Rotation Matrix', Q_R(quat))
                    print('TH final', fkine(q))
                    break
 
                derror = -k*e
                J_inv = np.linalg.inv(J.T.dot(J)).dot(J.T)
                dq = J_inv.dot(derror)
                q = q + dt*dq
 
                count = count + 1
 
                # print(np.linalg.norm(e))
 
                if(count > 10000):
                    print('Max number of iterations reached')
                    break
 
                # Current configuration trnaformation to current position
                T = fkine(q)
                x = TF2xyzquat(T)
                # Publish the message
                jstate.position = q
                pub.publish(jstate)
 
                # Wait for the next iteration
                rate.sleep()
 
            # Aqui acaba xd
 
            q0 = q
            deltax = [0, 0, 0]
 
        # Current time (needed for ROS)
        jstate.header.stamp = rospy.Time.now()
        # Add the head joint value (with value 0) to the joints
 
        #jstate.position = q
 
        # Publish the message
 
        #pub.publish(jstate)
        #bmarker.position(T)
        #bmarker.publish()
 
        # Wait for the next iteration
        rate.sleep()
