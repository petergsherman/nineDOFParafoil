#nineDOF_Aerodynamics.py
import numpy as np
from scipy.spatial.transform import Rotation

from nineDOF_Control import get_control
from nineDOF_Parameters import A_cradle, A_parafoil, C_D_cradle, b_bar_parafoil, c_bar_parafoil, d_bar_parafoil, deadband_parafoil, delta_nom, getInterpolatedAero, m_cradle, m_parafoil, x_pmp, y_pmp, z_pmp, CM0, CMQ, CMDS, CYB, CNB, CLB, CLP, CLR, CLDA, CNP, CNR, CND1, I_H, I_AM, I_AI, K_G, C_G, r_pmp
from nineDOF_Transform import skew, T_IP
from nineDOF_Atmosphere import getAirDensity


def construct_AM_Velocity(omega_PI, V_G):
    AM_Velocity = V_G + skew(omega_PI) @ r_pmp
    return AM_Velocity
    


def compute_aerodynamics(state, statedot, control):
    #Velocity Magnitude and Euler Angles
    V_mag = np.sqrt(state[9]**2 + state[10]**2 + state[11]**2)

    #Calculating Alpha and Beta
    alpha = np.arctan2(-state[11], state[9])
    beta = np.arctan2(state[10], np.sqrt(state[9]**2 + state[11]**2)) #////////////REVISIT////////////////

    #Getting Air Density
    rho = getAirDensity(state[2])

    #Getting Controls
    deltaL = control.get('delta_left', 0.0)
    deltaR = control.get('delta_right', 0.0)
    
    deltaA = (deltaR - deltaL) / d_bar_parafoil
    deltaS = ((0.5) * (deltaR + deltaL) - deadband_parafoil + delta_nom) / d_bar_parafoil


    #Calculating Aerodynamic Coefficients
    #Deflection and AOA Interpolated Aerodynamic Coefficients from Table Interpolation
    CD0, CDA2, CL0, CLA, CND2 = getInterpolatedAero(deltaS, alpha)

    #Non-Dimensional Angular Rates
    p_bar = (state[12] * b_bar_parafoil) / (2 * V_mag)
    q_bar = (state[13] * c_bar_parafoil) / (2 * V_mag)
    r_bar = (state[14] * b_bar_parafoil) / (2 * V_mag)

    #Aerodynamic Force Coefficients
    CD = CD0 + (CDA2 * (alpha**2))
    CY = CYB * beta
    CL = CL0 + (CLA * alpha)

    #Aerodynamic Moment Coefficients
    Cl = (CLB * beta) + (CLP * p_bar) + (CLR * r_bar) + (CLDA * deltaA)
    Cm = CM0 + (CMQ * q_bar) + (CMDS * deltaS)
    Cn = (CNB * beta) + (CNP * p_bar) + (CNR * r_bar) + (CND1 * (deltaL / d_bar_parafoil)) - (CND2 * ((deltaL * deltaS) / d_bar_parafoil)) + (CND1 * (deltaR / d_bar_parafoil)) + (CND2 * ((deltaR * deltaS) / d_bar_parafoil))

    #Aerodynamic Astate[12]arent Mass Velocity Calculation
    V_G = np.array([state[9], state[10], state[11]])
    omega_PI = T_IP(state[3], state[4], state[5]).T @ np.array([state[12], state[13], state[14]]) #???????????????????????????
    AM_velocity = construct_AM_Velocity(omega_PI, V_G)

    #Aerodynamic Astate[12]arent Mass Velocity Derivative Calculation
    V_G_dot = np.array([statedot[9], statedot[10], statedot[11]])
    omega_PI_dot = T_IP(state[3], state[4], state[5]).T @ np.array([statedot[12], statedot[13], statedot[14]]) #???????????????????????
    AM_Velocity_dot = construct_AM_Velocity(omega_PI_dot, V_G_dot)

    #Force Calculations 
    Fa_parafoil = (0.5) * rho * A_parafoil * (V_mag**2) * np.array([[(-np.cos(alpha) * CD) + (np.sin(alpha) * CL)],
                                                                    [CY],
                                                                    [(np.sin(alpha) * CD) + (np.cos(alpha) * CL)]])

    Fam_parafoil = (I_AM @ AM_Velocity_dot) + (skew(omega_PI) @ I_AM @ AM_velocity) # + (I_H @ np.array([state[12]_dot, state[13]_dot, state[14]_dot]))  + (skew(omega_PI) @ I_H @ np.array([state[12], state[13], state[14]])) since I_H == 0

    Fg_parafoil = m_parafoil * (9.81) * np.array([-np.sin(state[4])],
                                                 [np.sin(state[3]) * np.cos(state[3])],
                                                 [np.cos(state[3]) * np.cos(state[4])])  

    Fa_cradle = (0.5) * rho * (V_mag**2) * A_cradle * C_D_cradle * np.array([state[9],
                                                                             state[10],
                                                                             state[11]])       
    
    Fg_cradle = m_cradle * (9.81) * np.array([-np.sin(state[7])],
                                             [np.sin(state[6]) * np.cos(state[6])],
                                             [np.cos(state[6]) * np.cos(state[7])]) 
    #Moment Calculations
    Ma_parafoil = (0.5) * rho * A_parafoil * (V_mag**2) * np.array([[b_bar_parafoil * Cl],
                                                                    [c_bar_parafoil * Cm],
                                                                    [b_bar_parafoil * Cn]])

    Mam_parafoil = I_AI @ omega_PI_dot + (skew(omega_PI) @ I_AI) @ np.array([state[12], state[13], state[14]]) # + I_H @ AM_Velocity_dot + skew(omega_PI) @ I_H @ AM_Velocity + V_M_PI blah blah blah becasue I_H == 0

    M_gimbal = np.array([0,
                         0,
                         K_G(state[5] - state[8]) + C_G(statedot[5] - statedot[8])])

    return Fa_parafoil, Fam_parafoil, Fg_parafoil, Fa_cradle, Fg_cradle, Ma_parafoil, Mam_parafoil, M_gimbal