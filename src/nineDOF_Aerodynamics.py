#nineDOF_Aerodynamics.py
import numpy as np

from nineDOF_Control import get_controller
from nineDOF_Parameters import A_cradle, A_parafoil, C_D_cradle, b_bar_parafoil, c_bar_parafoil, d_bar_parafoil, deadband_parafoil, delta_nom, gamma_nom, getInterpolatedAero, m_cradle, m_parafoil, CM0, CMQ, CMDS, CYB, CNB, CLB, CLP, CLR, CLDA, CNP, CNR, CND1, r_GAp_P
from nineDOF_Transform import skew, makeT_PPI
from nineDOF_Atmosphere import getAirDensity

def compute_aerodynamics(state, control):

    #Velocity Magnitude
    vG_P = state[9:12]
    vAp_P = vG_P + skew(state[12:15]) @ r_GAp_P
    vAp_PI = makeT_PPI(0, gamma_nom).T @ vAp_P
    V_mag = np.linalg.norm(vAp_PI)

    #Calculating Alpha and Beta
    alpha = np.arctan2(vAp_PI[2], vAp_PI[0]) #//////////////////////make sure to use relative velocity based off of wind///////////////////
    beta = np.arcsin(vAp_PI[1] / V_mag) if abs(V_mag) > 1e-3 else 0 

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
    V_mag = max(V_mag, 1) #/////////////////////////////////REVISIT//////////////////////////////////
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

    #Force Calculations 
    Fa_parafoil_incidenceF = (0.5) * rho * A_parafoil * (V_mag**2) * np.array([(-np.cos(alpha) * CD) + (np.sin(alpha) * CL),
                                                                    CY,
                                                                    (-np.sin(alpha) * CD) - (np.cos(alpha) * CL)])
                                                                    
    Fa_parafoil = makeT_PPI(0, gamma_nom) @ Fa_parafoil_incidenceF #Rotate to Parafoil Frame

    Fg_parafoil = m_parafoil * (-9.81) * np.array([-np.sin(state[4]),
                                                  np.sin(state[3]) * np.cos(state[4]),
                                                  np.cos(state[3]) * np.cos(state[4])])  

    Fa_cradle = (-0.5) * rho * (V_mag) * A_cradle * C_D_cradle * np.array([state[9],
                                                                          state[10],
                                                                          state[11]])       
    
    Fg_cradle = m_cradle * (-9.81) * np.array([-np.sin(state[7]),
                                              np.sin(state[6]) * np.cos(state[7]),
                                              np.cos(state[6]) * np.cos(state[7])]) 
    #Moment Calculations
    Ma_parafoil = (0.5) * rho * A_parafoil * (V_mag**2) * np.array([b_bar_parafoil * Cl,
                                                                    c_bar_parafoil * Cm,
                                                                    b_bar_parafoil * Cn])

    return Fa_parafoil, Fg_parafoil, Fa_cradle, Fg_cradle, Ma_parafoil