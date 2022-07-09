import rbdl
import numpy as np
 
# Lectura del modelo del robot a partir de URDF (parsing)
modelo = rbdl.loadModel('../urdf/ur5_robot.urdf')
# Grados de libertad
ndof = modelo.q_size
 
# Configuracion articular
q = np.array([0.5, 0.2, 0.3, 0.8, 0.5, 0.6])
# Velocidad articular
dq = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0])
# Aceleracion articular
ddq = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5])
 
# Arrays numpy
zeros = np.zeros(ndof)          # Vector de ceros
tau   = np.zeros(ndof)          # Para torque
tau2   = np.zeros(ndof)         # Para torque
 
g   = np.zeros(ndof)        # Para la gravedad
c   = np.zeros(ndof)        # Para el vector de Coriolis+centrifuga
M   = np.zeros([ndof, ndof])  # Para la matriz de inercia
e   = np.eye(6)             # Vector identidad
 
# Torque dada la configuracion del robot
# rbdl.InverseDynamics(modelo, q, dq, ddq, tau)
 
# Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
# y matriz M usando solamente InverseDynamics
 
# vector g
rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
 
# vector c
rbdl.InverseDynamics(modelo, q, dq, zeros, c)
c = c - g
 
# matriz M
for i in range(ndof):
    rbdl.InverseDynamics(modelo, q, zeros, e[i,:], m_row)
    M[i,:] = m_row - g
 
print '\n vector g: \n', np.round(g,2)
print '\n vector c: \n', np.round(c,2)
print '\n matriz M: \n', np.round(M,2)
 
# Parte 2: Calcular M y los efectos no lineales b usando las funciones
# CompositeRigidBodyAlgorithm y NonlinearEffects. Almacenar los resultados
# en los arreglos llamados M2 y b2
b2 = np.zeros(ndof)         # Para efectos no lineales
M2 = np.zeros([ndof, ndof])  # Para matriz de inercia
 
# Confirmacion de M
rbdl.CompositeRigidBodyAlgorithm(modelo,q,M2,1)
print '\n matriz M: \n', np.round(M2,2)
 
# confirmacion de b
rbdl.NonlinearEffects(modelo,q,dq,b2)
print '\n matriz b: \n', np.round(b2,2)
 
# Parte 3: Verificacion de valores
print 'Diferencia de M y M2 \n', np.round(M-M2,2)
 
c2 = np.zeros(ndof)         # Para el vector de Coriolis+centrifuga
rbdl.InverseDynamics(modelo, q, dq, zeros, c2)
print 'Diferencia de b2 y c2 \n', np.round(b2-c2,2)
 
# Parte 4: Verificacion de la expresion de la dinamica
 
# Tau con ID
rbdl.InverseDynamics(modelo, q, dq, ddq, tau)
 
# Tau con CRBA y NLE
tau2 = M2.dot(ddq)+b2
 
print 'De ID: \n', np.round(tau,2)
print 'De CRBA y NLE: \n', np.round(tau2,2)
 
print 'Diferencia: \n', np.round(tau - tau2,2)#!/usr/bin/env python
 
import rospy
from sensor_msgs.msg import JointState
from markers import *
from functions import *
from roslib import packages
 
import rbdl
 
 
rospy.init_node("control_pdg")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker_actual  = BallMarker(color['RED'])
bmarker_deseado = BallMarker(color['GREEN'])
# Archivos donde se almacenara los datos
fqact = open("/tmp/qactual.txt", "w")
fqdes = open("/tmp/qdeseado.txt", "w")
fxact = open("/tmp/xactual.txt", "w")
fxdes = open("/tmp/xdeseado.txt", "w")
 
# Nombres de las articulaciones
jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Valores del mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
 
# =============================================================
# Configuracion articular inicial (en radianes)
q = np.array([0.0, -1.0, 1.7, -2.2, -1.6, 0.0])
# Velocidad inicial
dq = np.array([0., 0., 0., 0., 0., 0.])
# Aceleracion inicial
ddq = np.array([0., 0., 0., 0., 0., 0.])
# Configuracion articular deseada
qdes = np.array([1.0, -1.0, 1.0, 1.3, -1.5, 1.0])
# Velocidad articular deseada
dqdes = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# Aceleracion articular deseada
ddqdes = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
 
xdes = np.array([1,1,1])
dxdes = np.array([0,0,0])
ddxdes = np.array([0,0,0])
 
# =============================================================
 
# Posicion resultante de la configuracion articular deseada
xdes = ur5_fkine(qdes)[0:3,3]
# Copiar la configuracion articular en el mensaje a ser publicado
jstate.position = q
pub.publish(jstate)
 
# Modelo RBDL
modelo = rbdl.loadModel('../urdf/ur5_robot.urdf')
ndof   = modelo.q_size     # Grados de libertad
zeros = np.zeros(ndof)     # Vector de ceros
 
# Frecuencia del envio (en Hz)
freq = 20
dt = 1.0/freq
rate = rospy.Rate(freq)
 
# Simulador dinamico del robot
robot = Robot(q, dq, ndof, dt)
 
 
b = np.zeros(ndof)          # Para efectos no lineales
M = np.zeros([ndof, ndof])
c = np.zeros(ndof) 
 
Jp = np.zeros([ndof, ndof])
 
 
# Bucle de ejecucion continua
t = 0.0
 
# Se definen las ganancias del controlador
valores = 0.1*np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
Kp = np.diag(valores)
Kd = 2*np.sqrt(Kp)
 
while not rospy.is_shutdown():
 
    # Leer valores del simulador
    q  = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    # Posicion actual del efector final
    x = ur5_fkine(q)[0:3,3]
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()
 
    # Almacenamiento de datos
    fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
    fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
    fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+'\n ')
    fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' '+ str(qdes[2])+' '+ str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+'\n ')
 
    # ----------------------------
    # Control dinamico (COMPLETAR)
    # ----------------------------
 
    rbdl.CompositeRigidBodyAlgorithm(modelo,q,M,1)
    rbdl.InverseDynamics(modelo, q, dq, zeros, c)
    rbdl.NonlinearEffects(modelo,q,dq,b)
 
    J = jacobian_position(q,0.0001)
    dJ = (J - Jp)/dt
 
    u = M*np.linalg.pinv(J)*(ddxdes - dJ*dq + Kd*(dxdes - J*dq) + Kp*(xdes - x) + b)
 
    Jp = J
 
    # Simulacion del robot
    robot.send_command(u)
 
    # Publicacion del mensaje
    jstate.position = q
    pub.publish(jstate)
    bmarker_deseado.xyz(xdes)
    bmarker_actual.xyz(x)
    t = t+dt
    # Esperar hasta la siguiente  iteracion
    rate.sleep()
 
fqact.close()
fqdes.close()
fxact.close()
fxdes.close()
