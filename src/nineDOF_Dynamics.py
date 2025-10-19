#nineDOF_Dynamics.
import numpy as np

from nineDOF_Aerodynamics import compute_aerodynamics
from nineDOF_Parameters import m_parafoil, m_cradle, I_parafoil, I_cradle, I_AM, I_AI, I_H, K_G, C_G, r_GC_C, r_CG_C, r_PG_P, r_PMp_P, r_PAp_P, r_GMp_P
from nineDOF_Transform import skew, makeT_IP, makeT_IC, makeH


def compute_dynamics(state, control):
    #Statedot Vector Creation
    statedot = np.zeros(18)

    #Getting Aerodynamic Forces and Moments
    Fa_parafoil, Fg_parafoil, Fa_cradle, Fg_cradle, Ma_parafoil = compute_aerodynamics(state, control)

    #Computing Transformation Matrices
    T_IP = makeT_IP(state[3:6])
    T_IC = makeT_IC(state[6:9])
    H_P = makeH(state[12:15])
    H_C = makeH(state[15:18])
    T_PC = T_IP.T @ T_IC
    T_CP = T_IC.T @ T_IP

    #Computing Kinematic Derivatives
    statedot[0:3] = T_IP @ state[9:12]
    statedot[3:6] = H_P @ state[12:15] #///////////////REVISIST//////////////////////
    statedot[6:9] = H_C @ state[15:18]

    #Computing Gimbal Moment
    M_gimbal = np.array([0,
                         0,
                         K_G(state[5] - state[8]) + C_G(statedot[5] - statedot[8])])
    
    #Constructing Vectors for Readablitiy
    vG_P = state[9:12]
    wp_P = state[12:15]
    wc_C = state[15:18]
    vMp_P = vG_P + skew(wp_P @ r_GMp_P) #Velocity of Apparent Mass Center 

    #Calculating A Matrix 
    A11 = m_parafoil * skew(r_CG_C)
    A12 = 0
    A13 = m_cradle * T_IC.T @ T_IP
    A14 = -T_IC.T

    A21 = 0
    A22 = -I_AM @ skew(r_GMp_P) + I_H + (m_parafoil * skew(r_PG_P))
    A23 = (m_parafoil * np.eye(3)) + I_AM #////////////REVISIST COMPARED TO FORTRAN///////////////
    A24 = T_IP.T

    A31 = I_cradle
    A32 = 0
    A33 = 0
    A34 = -skew(r_CG_C) @ T_IC.T
    
    A41 = 0
    A42 = I_parafoil + skew(r_PMp_P) @ (I_H - I_AM @ skew(r_GMp_P)) - I_H @ skew(r_GMp_P) + I_AI
    A43 = I_H + skew(r_PMp_P) @ I_AM
    A44 = skew(r_PG_P) @ T_IP.T

    A = np.array([[A11, A12, A13, A14],
                  [A21, A22, A23, A24],
                  [A31, A32, A33, A34],
                  [A41, A42, A43, A44]])

    #Calculating B Vector
    B1 = Fa_cradle + Fg_cradle \
         - m_cradle * T_CP @ (skew(wp_P) @ vG_P) \
         - m_cradle * wc_C @ (skew(wc_C) @ r_GC_C)

    B2 = Fa_parafoil + Fg_parafoil \
         - m_parafoil * (skew(wp_P) @ vG_P) \
         - (skew(wp_P) @ ((I_AM @ vMp_P) + (I_H @ wp_P))) \
         + m_parafoil * (skew(wp_P) @ (skew(wp_P) @ r_PG_P))

    B3 = (T_CP @ M_gimbal) - (skew(wc_C) @ (I_cradle @ wc_C)) #No Tether Included

    B4 = Ma_parafoil + (skew(r_PAp_P) @ Fa_parafoil) - M_gimbal \
         - (((skew(r_PMp_P) @ skew(wp_P)) @ (skew(vMp_P) @ I_H) @ (skew(wp_P) @ (I_AI + I_parafoil))) @ wp_P) \
         - (((skew(r_PMp_P) @ (skew(wp_P) @ I_AM)) + (skew(wp_P) @ I_H)) @ vMp_P)

    #Stack into 12×1 (flatten to 12,)
    B = np.concatenate([B1, B2, B3, B4], axis = 0)

    #Solve A x = B
    try:
        x = np.linalg.solve(A,B)
    except np.linalg.linAlgError:
        #If A is singular/ill-conditioned in some poses, fall back to least-squares
        x = np.linalg.lstsq(A, B, rcond = None)[0]
    
    #Updating Statedot Vector
    statedot[15:18] = x[0:3]   # cradle angular velocity dynamics
    statedot[12:15] = x[3:6]   # parafoil angular velocity dynamics
    statedot[9:12]  = x[6:9]   # gimbal joint velocity dynamics

    return statedot