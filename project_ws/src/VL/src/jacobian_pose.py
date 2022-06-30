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