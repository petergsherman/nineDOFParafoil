#nineDOF_Aerodynamics.py
import numpy as np
from scipy.spatial.transform import Rotation

from nineDOF_Control import get_control
from nineDOF_Parameters import A_cradle, A_parafoil, C_D_cradle, b_bar_parafoil, c_bar_parafoil, d_bar_parafoil, deadband_parafoil, delta_nom, getInterpolatedAero, m_cradle, m_parafoil, x_pmp, y_pmp, z_pmp, CM0, CMQ, CMDS, CYB, CNB, CLB, CLP, CLR, CLDA, CNP, CNR, CND1
from nineDOF_Transform import skew









def compute_aerodynamics(state, statedot, control):
    #Getting State
    uG, vG, wG = state['uG'], state['vG'], state['wG'] #Translation Velocities from Current State
    pP, qP, rP = state['pP'], state['qP'], state['rP'] #Angular Velocities of Parafoil
    pC, qC, rC = state['pC'], state['qC'], state['rC'] #Angular Velocities of Cradle
    p_quat = state['p_quat'] #Parafoil Quaternion
    c_quat = state['c_quat'] #Cradle Quaternion

    #Getting State Derivatives
    uG_dot, vG_dot, wG_dot = statedot['uG_dot'], statedot['vG_dot'], statedot['wG_dot']
    pP_dot, qP_dot, rP_dot = statedot['pP_dot'], statedot['qP_dot'], statedot['rP_dot']
    pC_dot, qC_dot, rC_dot = statedot['pC_dot'], statedot['qC_dot'], statedot['rC_dot']

    V_mag = np.sqrt(uG**2 + vG**2 + wG**2)
    p_phi, p_theta, p_psi = Rotation.from_quat(p_quat).as_euler('xyz', degrees = False)
    c_phi, c_theta, c_psi = Rotation.from_quat(c_quat).as_euler('xyz', degrees = False)

    #Calculating Alpha and Beta
    



    #Getting Controls
    deltaL = control.get('delta_left', 0.0)
    deltaR = control.get('delta_right', 0.0)
    
    deltaA = (deltaR - deltaL) / d_bar_parafoil
    deltaS = ((0.5) * (deltaR + deltaL) - deadband_parafoil + delta_nom) / d_bar_parafoil


    #Calculating Aerodynamic Coefficients
    #Deflection and AOA Interpolated Aerodynamic Coefficients
    CD0, CDA2, CL0, CLA, CND2 = getInterpolatedAero(deltaS, alpha)

    #Non-Dimensional Angular Rates
    p_bar = (pP * b_bar_parafoil) / (2 * V_mag)
    q_bar = (qP * c_bar_parafoil) / (2 * V_mag)
    r_bar = (rP * b_bar_parafoil) / (2 * V_mag)

    #Aerodynamic Force Coefficients
    CD = CD0 + (CDA2 * (alpha**2))
    CY = CYB * beta
    CL = CL0 + (CLA * alpha)

    #Aerodynamic Moment Coefficients
    Cl = (CLB * beta) + (CLP * p_bar) + (CLR * r_bar) + (CLDA * deltaA)
    Cm = CM0 + (CMQ * q_bar) + (CMDS * deltaS)
    Cn = (CNB * beta) + (CNP * p_bar) + (CNR * r_bar) + (CND1 * (deltaL / d_bar_parafoil)) - (CND2 * ((deltaL * deltaS) / d_bar_parafoil)) + (CND1 * (deltaR / d_bar_parafoil)) + (CND2 * ((deltaR * deltaS) / d_bar_parafoil))

    #Force Calculations 
    Fa_parafoil = (0.5) * rho * A_parafoil * (V_mag**2) * np.array([[(-np.cos(alpha) * CD) + (np.sin(alpha) * CL)],
                                                                    [CY],
                                                                    [(np.sin(alpha) * CD) + (np.cos(alpha) * CL)]])

    Fg_parafoil = m_parafoil * (9.81) * np.array([-np.sin(p_theta)],
                                                 [np.sin(p_phi) * np.cos(p_phi)],
                                                 [np.cos(p_phi) * np.cos(p_theta)])  

    Fa_cradle = (0.5) * rho * (V_mag**2) * A_cradle * C_D_cradle * np.array([uG,
                                                                             vG,
                                                                             wG])              
    
    Fg_cradle = m_cradle * (9.81) * np.array([-np.sin(c_theta)],
                                             [np.sin(c_phi) * np.cos(c_phi)],
                                             [np.cos(c_phi) * np.cos(c_theta)]) 
    #Moment Calculations
    Ma_parafoil = (0.5) * rho * A_parafoil * (V_mag**2) * np.array([[b_bar_parafoil * Cl],
                                                                    [c_bar_parafoil * Cm],
                                                                    [b_bar_parafoil * Cn]])