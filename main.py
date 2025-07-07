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
    A = 0.2 # Amplitude
    f = 0.5 # Hz
    w = 2*pi*f

    # PID
    kp = -20
    kd = -10
    ki = 4 * pow(10, -1)

    x0 = 1
    v0 = 0
    z0 = 0

    h = 0.01
    T = 20
    N = round(T/h)
    
    t = [i*h for i in range(N+1)]
    theta = [A * sin(w * ti) for ti in t]
    eta = [A * sin(w * ti) for ti in t]

    x = [x0]    
    v = [v0]
    z = [z0]
    
    for k in range(0, N):
        tk = t[k]

        # K1
        k1_z = x[k]
        k1_x = v[k]
        k1_v = (g + kp * x[k] + kd * v[k] + ki * z[k]) * cos(theta[k]) * cos(eta[k]) - g

        # K2
        theta_k2 = np.interp(tk + h/2, t, theta)
        eta_k2 = np.interp(tk + h/2, t, eta)
        x_temp2 = x[k] + h/2 * k1_x
        v_temp2 = v[k] + h/2 * k1_v
        z_temp2 = z[k] + h/2 * k1_z

        k2_z = x_temp2
        k2_x = v_temp2
        k2_v = (g + kp * x_temp2 + kd * v_temp2 + ki * z_temp2) * cos(theta_k2) * cos(eta_k2) - g

        # K3
        theta_k3 = np.interp(tk + h/2, t, theta)
        eta_k3 = np.interp(tk + h/2, t, eta)
        x_temp3 = x[k] + h/2 * k2_x
        v_temp3 = v[k] + h/2 * k2_v
        z_temp3 = z[k] + h/2 * k2_z

        k3_z = x_temp3
        k3_x = v_temp3
        k3_v = (g + kp * x_temp3 + kd * v_temp3 + ki * z_temp3) * cos(theta_k3) * cos(eta_k3) - g

        # K4
        theta_k4 = np.interp(tk + h, t, theta)
        eta_k4 = np.interp(tk + h, t, eta)
        x_temp4 = x[k] + h * k3_x
        v_temp4 = v[k] + h * k3_v
        z_temp4 = z[k] + h * k3_z

        k4_z = x_temp4
        k4_x = v_temp4
        k4_v = (g + kp * x_temp4 + kd * v_temp4 + ki * z_temp4) * cos(theta_k4) * cos(eta_k4) - g

        # Iteração
        z.append(z[k] + h/6 * (k1_z + 2*k2_z + 2*k3_z + k4_z))
        x.append(x[k] + h/6 * (k1_x + 2*k2_x + 2*k3_x + k4_x))
        v.append(v[k] + h/6 * (k1_v + 2*k2_v + 2*k3_v + k4_v))

    plt.plot(t, x)
    plt.xlabel('Tempo (s)')
    plt.ylabel('Posição x (m)')
    plt.title('Altura vs tempo')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    pid()