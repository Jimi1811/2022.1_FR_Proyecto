from copy import copy
import numpy as np
# import rbdl

pi = np.pi


# class Robot(object):
#     def __init__(self, q0, dq0, ndof, dt):
#         self.q = q0    # numpy array (ndof x 1)
#         self.dq = dq0  # numpy array (ndof x 1)
#         self.M = np.zeros([ndof, ndof])
#         self.b = np.zeros(ndof)
#         self.dt = dt
#         self.robot = rbdl.loadModel('../urdf/ur5_robot.urdf')

#     def send_command(self, tau):
#         rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
#         rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
#         ddq = np.linalg.inv(self.M).dot(tau-self.b)
#         self.q = self.q + self.dt*self.dq
#         self.dq = self.dq + self.dt*ddq

#     def read_joint_positions(self):
#         return self.q

#     def read_joint_velocities(self):
#         return self.dq

# ----------------------------------
# Matriz de transformacion homogenea

def dh(d, theta, a, alpha):
    """
    Matriz de transformacion homogenea asociada a los parametros DH.
    Retorna una matriz 4x4
    """

    sth = np.sin(theta)
    cth = np.cos(theta)
    sa = np.sin(alpha)
    ca = np.cos(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                  [sth,  ca*cth, -sa*cth, a*sth],
                  [0.0,      sa,      ca,     d],
                  [0.0,     0.0,     0.0,   1.0]])
    return T

# ----------------------------------
# Cinematica directa


def fkine(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """

    # Matrices DH
    T1 = dh(  0.126223,       q[0],  0.035737, pi/2)
    T2 = dh(    0.0175,  q[1]-pi/2,         0, pi/2)
    T3 = dh(    -0.534,       q[2],         0, pi/2)
    T4 = dh(    0.0995,       q[3],         0, pi/2)
    T5 = dh(0.355+q[4],          0,         0,   pi)
    T6 = dh(    -0.029,       q[5],         0,    0)

    # Efector final con respecto a la base
    T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)

    return T

# ----------------------------------
# Jacobiano de posicion


def jacobian_position(q):
    delta = 0.0001

    # Crear una matriz 3x6
    J = np.zeros((3, 6))
    # Transformacion homogenea inicial (usando q)
    To = fkine(q)
    To = To[0:3, -1:]  # vector posicion

    # Iteracion para la derivada de cada columna
    for i in range(6):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i] = dq[i]+delta
        # Transformacion homogenea luego del incremento (q+delta)
        T = fkine(dq)
        T = T[0:3, -1:]  # vector posicion
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        Jq = 1/delta*(T-To)
        J[:, i:i+1] = Jq

    return J

# ----------------------------------
# Cinematica inversa


def ikine(xdes, q0):
    # Error
    epsilon = 0.001
    # Maximas iteraciones
    max_iter = 1000
    # Delta de la jacobiana
    delta = 0.00001
    # Copia de las articulaciones
    q = copy(q0)
    # Almacenamiento del error
    ee = []
    # Transformacion homogenea (usando q)
    To = fkine(q)
    To = To[0:3, 3]  # vector posicion
    # Resetear cuando se llega a la cantidad maxima de iteraciones
    restart = True

    while restart:
        for i in range(max_iter):
            # Hacer el for 1 vez
            restart = False
            # Pseudo-inversa del jacobiano
            J = jacobian_position(q)
            J = np.linalg.pinv(J)
            # Error entre el x deseado y x actual
            e = xdes - To
            # q_k+1
            q = q + np.dot(J, e)
            # Nueva mtransformada homogenea
            To = fkine(q)
            To = To[0:3, 3]  # vector posicion

            # Norma del error
            enorm = np.linalg.norm(e)
            ee.append(enorm)    # Almacena los errores
            # Condicion de termino
            if (enorm < epsilon):
                print("Error en la iteracion ", i, ": ", np.round(enorm, 4))
                break
            if (i == max_iter-1 and enorm > epsilon):
                print("Iteracion se repite")
                print("Error en la iteracion ", i, ": ", enorm)
            restart = True
    return q

# -------------------------------
# Control por cinematica diferencial

# Jacobiano de posicion y orientacion como cuaternion
def jacobian_pose(q, delta):
    """
    Jacobiano analitico para la posicion y orientacion (usando un
    cuaternion). Retorna una matriz de 7x6 y toma como entrada el vector de
    configuracion articular q=[q1, q2, q3, q4, q5, q6]
    
    """
 
    J = np.zeros((7,6))
    # Implementar este Jacobiano aqui
    To = fkine(q)
    x0 = TF2xyzquat(To)
 
    # Iteracion para la derivada de cada columna
    for i in range(6):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i] = dq[i]+delta
        # Transformacion homogenea luego del incremento (q+delta)
        T = fkine(dq)
        x = TF2xyzquat(T)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        Jq = 1/delta*(x-x0)
 
        J[0, i:i+1] = Jq[0]
        J[1, i:i+1] = Jq[1]
        J[2, i:i+1] = Jq[2]
        J[3, i:i+1] = Jq[3]
        J[4, i:i+1] = Jq[4]
        J[5, i:i+1] = Jq[5]
        J[6, i:i+1] = Jq[6]
 
    return J

# Conversion de matriz de rotacion a cuaternion
def rot2quat(R):
    """
    Convertir una matriz de rotacion en un cuaternion

    Entrada:
      R -- Matriz de rotacion
    Salida:
      Q -- Cuaternion [ew, ex, ey, ez]

    """
    dEpsilon = 1e-6
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)

    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)

    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)

    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return np.array(quat)

# Conversion de matriz T a vector con possicion y cuaternion
def TF2xyzquat(T):
    """
    Convert a homogeneous transformation matrix into the a vector containing the
    pose of the robot.

    Input:
      T -- A homogeneous transformation
    Output:
      X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
           is Cartesian coordinates and the last part is a quaternion
    """
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)

# Conversion de matriz antrisimetrica a de rotacion
def skew(w):
    R = np.zeros([3,3])
    R[0,1] = -w[2]; R[0,2] = w[1]
    R[1,0] = w[2];  R[1,2] = -w[0]
    R[2,0] = -w[1]; R[2,1] = w[0]
    return R
 
# Matriz de rotacion R a partir de Q
def Q_R(Q):
    w = Q[0]; ex = Q[1]; ey = Q[2]; ez = Q[3]
    R = np.array([
        [2*(w**2+ex**2)-1,   2*(ex*ey-w*ez),    2*(ex*ez+w*ey)],
        [  2*(ex*ey+w*ez), 2*(w**2+ey**2)-1,    2*(ey*ez-w*ex)],
        [  2*(ex*ez-w*ey),   2*(ey*ez+w*ex),    2*(w**2+ez**2)-1]
    ])
    return R