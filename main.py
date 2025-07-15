import math
import matplotlib
matplotlib.use('TkAgg')   
import matplotlib.pyplot as plt
import numpy as np
pi = math.pi 
sin = math.sin
cos = math.cos

def pid():
    g = 9.81 # m/s²
    m1 = 0.5 # kg
    m2 = 0.7 # kg
    M = 3 # kg
    L = 1.5 # metros
    d = 0.05 # metros
    L1 = L/2 - d
    L2 = L/2 + d
    Ixx = 1/12 * M * L**2 # kg.m²
    J = Ixx + M*d**2 + m1*L1**2 + m2*L2**2 # Momento de inércia da barra + motores em relação ao eixo x
    B = 0.2 # Coeficiente de amortecimento em [vou pesquisar a unidade]

    # PID
    kp = 5
    kd = 5
    ki = 0.0001
    # ziegler nichols

    x0 = -30 # Angulo inicial
    v0 = 0
    z0 = 0

    h = 0.01 # Passo
    T = 20
    N = round(T/h)
    
    t = [i*h for i in range(N+1)]

    x = [x0]    
    v = [v0]
    z = [z0]
    
    for k in range(0, N):
        #tk = t[k]

        # K1
        k1_z = x[k]
        k1_x = v[k]
        k1_v = -(kp * x[k] + (kd + B) * v[k] + ki * z[k])/J

        # K2
        x_temp2 = x[k] + h/2 * k1_x
        v_temp2 = v[k] + h/2 * k1_v
        z_temp2 = z[k] + h/2 * k1_z

        k2_z = x_temp2
        k2_x = v_temp2
        k2_v = -(kp * x_temp2 + (kd + B) * v_temp2 + ki * z_temp2)/J

        # K3
        x_temp3 = x[k] + h/2 * k2_x
        v_temp3 = v[k] + h/2 * k2_v
        z_temp3 = z[k] + h/2 * k2_z

        k3_z = x_temp3
        k3_x = v_temp3
        k3_v = -(kp * x_temp3 + (kd + B) * v_temp3 + ki * z_temp3)/J

        # K4
        x_temp4 = x[k] + h * k3_x
        v_temp4 = v[k] + h * k3_v
        z_temp4 = z[k] + h * k3_z

        k4_z = x_temp4
        k4_x = v_temp4
        k4_v = -(kp * x_temp4 + (kd + B) * v_temp4 + ki * z_temp4)/J

        # Iteração
        z.append(z[k] + h/6 * (k1_z + 2*k2_z + 2*k3_z + k4_z))
        x.append(x[k] + h/6 * (k1_x + 2*k2_x + 2*k3_x + k4_x))
        v.append(v[k] + h/6 * (k1_v + 2*k2_v + 2*k3_v + k4_v))

    plt.plot(t, x)
    plt.xlabel('Tempo (s)')
    plt.ylabel('Angulo (°)')
    plt.title('Angulo vs tempo')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    pid()