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
    L_medio = L/2 # Equivalente à média de L1 e L2
    Ixx = 1/12 * M * L**2 # kg.m²
    J = Ixx + M*d**2 + m1*L1**2 + m2*L2**2 # Momento de inércia da barra + motores em relação ao eixo x
    B = 0.2 # Coeficiente de amortecimento em [vou pesquisar a unidade]

    W = M * g
    W1 = m1 * g
    W2 = m2 * g
    T_Pesos = W * d + W1*L1 - W2*L2

    # PID
    kp = 0
    kd = 0
    ki = 0
    # ziegler nichols

    phi0 = -pi/6 # Angulo inicial em radianos
    dphi0 = 0 # Velocidade angular em rad/s
    z0 = 0 # Erro inicial
    Tm0 = 0 # Torque inicial dos motores
    T0 = -cos(phi0) * T_Pesos - B * dphi0 # Torque inicial do sistema

    h = 0.01 # Passo
    T = 20
    N = round(T/h)

    t = [i*h for i in range(N+1)]

    phi = [phi0]    
    dphi = [dphi0]
    z = [z0]
    Torque_motores = [Tm0]
    Torque_total = [T0]

    def Torque(phi_k, dphi_k, z_k):
        T_motores = -(kp * phi_k + kd * dphi_k + ki * z_k) * L_medio
        T_total = T_motores - cos(phi_k) * T_Pesos - B * dphi_k
        return T_motores, T_total

    for k in range(0, N):

        # K1
        k1_z = phi[k]
        k1_phi = dphi[k]
        T_motores_k1, T_total_k1 = Torque(phi[k], dphi[k], z[k])
        k1_dphi = T_total_k1 / J

        # K2
        phi_temp2 = phi[k] + h/2 * k1_phi
        dphi_temp2 = dphi[k] + h/2 * k1_dphi
        z_temp2 = z[k] + h/2 * k1_z
        T_motores_k2, T_total_k2 = Torque(phi_temp2, dphi_temp2, z_temp2)

        k2_z = phi_temp2
        k2_phi = dphi_temp2
        k2_dphi = T_total_k2 / J

        # K3
        phi_temp3 = phi[k] + h/2 * k2_phi
        dphi_temp3 = dphi[k] + h/2 * k2_dphi
        z_temp3 = z[k] + h/2 * k2_z
        T_motores_k3, T_total_k3 = Torque(phi_temp3, dphi_temp3, z_temp3)

        k3_z = phi_temp3
        k3_phi = dphi_temp3
        k3_dphi = T_total_k3 / J

        # K4
        phi_temp4 = phi[k] + h * k3_phi
        dphi_temp4 = dphi[k] + h * k3_dphi
        z_temp4 = z[k] + h * k3_z
        T_motores_k4, T_total_k4 = Torque(phi_temp4, dphi_temp4, z_temp4)

        k4_z = phi_temp4
        k4_phi = dphi_temp4
        k4_dphi = T_total_k4 / J

        # Iteração
        z.append(z[k] + h/6 * (k1_z + 2*k2_z + 2*k3_z + k4_z))
        phi.append(phi[k] + h/6 * (k1_phi + 2*k2_phi + 2*k3_phi + k4_phi))
        dphi.append(dphi[k] + h/6 * (k1_dphi + 2*k2_dphi + 2*k3_dphi + k4_dphi))
        Torque_motores.append((T_motores_k1 + 2*T_motores_k2 + 2*T_motores_k3 + T_motores_k4) / 6)
        Torque_total.append((T_total_k1 + 2*T_total_k2 + 2*T_total_k3 + T_total_k4) / 6)
    
    Plot_Grafico(t, phi, Torque_total, Torque_motores)

def Plot_Grafico(t, phi, T_total, T_motores):
    fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    axs[0].plot(t, phi, color='blue')
    axs[0].set_ylabel('Ângulo (rad)')
    axs[0].set_title('Resposta angular e torque aplicado pelo PID')
    axs[0].grid(True)

    axs[1].plot(t, T_total, color='green')
    axs[1].plot(t, T_motores, color='red')
    axs[1].set_xlabel('Tempo (s)')
    axs[1].set_ylabel('Torque (N·m)')
    axs[1].grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    pid()