#nineDOF_Aerodynamics.py
import numpy as np
from nineDOF_Control import get_control
from nineDOF_Parameters import b_bar_parafoil, c_bar_parafoil, d_bar_parafoil, deadband_parafoil, delta_nom, getInterpolatedAero, num_panels, x_pmp, y_pmp, z_pmp, CM0, CMQ, CMDS, CYB, CNB, CLB, CLP, CLR, CLDA, CNP, CNR, CND1
from nineDOF_Transform import skew

from scipy.spatial.transform import Rotation









def compute_aerodynamics(state, control):
    #Getting State
    uG, vG, wG = state['uG'], state['vG'], state['wG'] #Translation Velocities from Current State
    pP, qP, rP = state['pP'], state['qP'], state['rP'] #Angular Velocities of Parafoil
    pC, qC, rC = state['pC'], state['qC'], state['rC'] #Angular Velocities of Cradle
    p_quat = state['p_quat'] #Parafoil Quaternion
    c_quat = state['c_quat'] #Cradle Quaternion

    V_mag = np.sqrt(uG**2 + vG**2 + wG**2)

    #Calculating Alpha and Beta
    p_phi, p_theta, p_psi = Rotation.from_quat(p_quat).as_euler('xyz', degrees = False)



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