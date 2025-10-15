#nineDOF_Dynamics.
import numpy as np
from scipy.spatial.transform import Rotation

from nineDOF_Aerodynamics import compute_aerodynamics, constructAMVelocity
from nineDOF_Parameters import m_parafoil, m_cradle, I_parafoil, I_cradle, I_AM, r_gc
from nineDOF_Transform import skew, T_IP, T_IC


def compute_dynamics(state, statedot, control):
    #Getting Aerodynamic Forces and Moments
    Fa_parafoil, Fam_parafoil, Fg_parafoil, Fa_cradle, Fg_cradle, Ma_parafoil, Mam_parafoil, M_gimbal = compute_aerodynamics(state, statedot, control)

    #Calculating A Matrix 
    A11 = m_parafoil * skew(r_gc)
    A12 = 0
    A13 = m_cradle * T_IC(state[6], state[7], state[8]) @ T_IP(state[3], state[4], state[5]).T
    A14 = -T_IC(state[6], state[7], state[8])

    A21 = 0
    A22 = -I_AM @ skew(r_)
    A23 = m_parafoil + I_AM
    A24 = T_IP(state[3], state[4], state[5])

    A31 = I_cradle
    A32 = 0
    A33 = 0
    A34 = -skew(r_pG) @ T_IC(state[6], state[7], state[8])
    
    A41 = 0
    A42 = I_parafoil + skew(r_pMp) @ (I_H - I_AM @ skew(r_GMp)) - I_H @ skew(r_GMp) + I_AI
    A43 = I_H + skew(r_pMp) @ I_AM
    A44 = skew(r_pG) @ T_IP(state[3], state[4], state[5])

    A = np.array([[A11, A12, A13, A14],
                  [A21, A22, A23, A24],
                  [A31, A32, A33, A34],
                  [A41, A42, A43, A44]])

    #Calculating B Vector
    B1 = Fa_cradle + Fg_cradle - m_cradle * T_IC(state[6], state[7], state[8]) @ T_I @ T_P @ skew(omega_PI) @ np.array(state[9], state[10], state[11]]) + m_cradle * skew(omega_CI) @ skew(omega_CI) @ np.array([xG, yG, zG])
    B2 =
    B3 =
    B4 =

    B = np.array([B1, 
                  B2, 
                  B3, 
                  B4])