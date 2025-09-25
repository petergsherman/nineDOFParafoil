#nineDOF_Dynamics.
import numpy as np
from nineDOF_Aerodynamics import compute_aerodynamics, constructAMVelocity
from nineDOF_Parameters import m_parafoil, m_cradle, I_parafoil, I_cradle
from nineDOF_Transform import skew, T_IP, T_IC

def compute_dynamics(state, statedot, control):
    #Getting State
    uG, vG, wG = state['uG'], state['vG'], state['wG'] #Translation Velocities from Current State
    pP, qP, rP = state['pP'], state['qP'], state['rP'] #Angular Velocities of Parafoil
    pC, qC, rC = state['pC'], state['qC'], state['rC'] #Angular Velocities of Cradle
    p_quat = state['p_quat'] #Parafoil Quaternion
    c_quat = state['c_quat'] #Cradle Quaternion

    p_phi, p_theta, p_psi = Rotation.from_quat(p_quat).as_euler('xyz', degrees = False)
    c_phi, c_theta, c_psi = Rotation.from_quat(c_quat).as_euler('xyz', degrees = False)

    #Getting State Derivatives
    uG_dot, vG_dot, wG_dot = statedot['uG_dot'], statedot['vG_dot'], statedot['wG_dot']
    pP_dot, qP_dot, rP_dot = statedot['pP_dot'], statedot['qP_dot'], statedot['rP_dot']
    pC_dot, qC_dot, rC_dot = statedot['pC_dot'], statedot['qC_dot'], statedot['rC_dot']

    #Getting Aerodynamic Forces and Moments
    Fa_parafoil, Fam_parafoil, Fg_parafoil, Fa_cradle, Fg_cradle, Ma_parafoil, Mam_parafoil, M_gimbal = compute_aerodynamics(state, statedot, control)

    #Calculating A Matrix 
    A11 =
    A12 = 0
    A13 = m_cradle * T_IC(c_phi, c_theta, c_psi) @ T_IP(p_phi, p_theta, p_psi).T
    A14 = -T_IC(c_phi, c_theta, c_psi)

    A21 = 0
    A22 = 
    A23 = m_parafoil + I_AM
    A24 = T_IP(p_phi, p_theta, p_psi)

    A31 = I_Cradle
    A32 = 0
    A33 = 0
    A34 = 
    
    A41 = 0
    A42 = 
    A43 = 
    A44 = 

    A = np.array([[A11, A12, A13, A14],
                  [A21, A22, A23, A24],
                  [A31, A32, A33, A34],
                  [A41, A42, A43, A44]])

    #Calculating B Vector
    B1 =
    B2 =
    B3 =
    B4 =

    B = np.array([B1, 
                  B2, 
                  B3, 
                  B4])