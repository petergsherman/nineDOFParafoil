#nineDOF_Dynamics.
import numpy as np
from scipy.spatial.transform import Rotation

from nineDOF_Aerodynamics import compute_aerodynamics, constructAMVelocity
from nineDOF_Parameters import m_parafoil, m_cradle, I_parafoil, I_cradle, I_AM, I_AI, I_H
from nineDOF_Transform import skew, makeT_IP, makeT_IC, makeH


def compute_dynamics(state, statedot, control):
    #Getting Aerodynamic Forces and Moments
    Fa_parafoil, Fam_parafoil, Fg_parafoil, Fa_cradle, Fg_cradle, Ma_parafoil, Mam_parafoil, M_gimbal = compute_aerodynamics(state, statedot, control)

    #Computing Transformation Matrices
    T_IP = makeT_IP(state[3], state[4], state[5])
    T_IC = makeT_IC(state[6], state[7], state[8])
    H_P = makeH(state[12], state[13], state[14])
    H_C = makeH(state[15], state[16], state[17])
    T_PC = T_IP.T @ T_IC
    T_CP = T_IC.T @ T_IP

    #Computing Kinematic Derivatives
    statedot[0:3] = T_IP @ state[9:12]
    statedot[3:6] = H_P @ state[12:15] #///////////////REVISIST//////////////////////
    statedot[6:9] = H_C @ state[15:18]
    
    #Calculating A Matrix 
    A11 = m_parafoil * skew(r_CG_C)
    A12 = 0
    A13 = m_cradle * T_IC.T @ T_IP
    A14 = -T_IC.T

    A21 = 0
    A22 = -I_AM @ skew(r_GMp_P) + I_H + (m_parafoil * skew(r_PG_P))
    A23 = m_parafoil + I_AM #////////////REVISIST COMPARED TO FORTRAN///////////////
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
    B1 = Fa_cradle + Fg_cradle - m_cradle * T_CP @ (skew(state[12:15]) @ state[9:12]) - m_cradle * state[15:18] @ (skew(state[15:18]) @ r_GC_C)
    B2 =
    B3 =
    B4 =

    B = np.array([B1, 
                  B2, 
                  B3, 
                  B4])