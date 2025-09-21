#nineDOF_Aerodynamics.py
import numpy as np
from nineDOF_Control import get_control
from nineDOF_Parameters import BL_panel, SL_panel, WL_panel, num_panels, x_pmp, y_pmp, z_pmp
from nineDOF_Transform import skew

def compute_aerodynamic_velocity(state, i):
    uG, vG, wG = state['uG'], state['vG'], state['wG'] #Translation Velocities from Current State
    pP, qP, rP = state['pP'], state['qP'], state['rP']
    r_PMP = np.array([x_pmp, y_pmp, z_pmp]) #Position Vector from Parafoil Center of Gravity to the Apparent Mass Center

    if i != -1: 
        rCG_panel = np.array([SL_panel[i], BL_panel[i], WL_panel[i]]) #Position Vector from the Center of Gravity to the ith Panel

    rMp_panel = rCG_panel - r_PMP #Position Vector from Apparent Mass Center to the ith panel
    omega_PI = np.array([pP, qP, rP]) #Angular Velocity of Parafoil with Respect to Intertial Frame
    skew_omega_PI = skew(omega_PI)
    aero_velocity = np.array([uG, vG, wG]) + skew_omega_PI @ rMp_panel #EQUATION 28

    return aero_velocity



def compute_aerodynamics(state, control):
    uG, vG, wG = state['uG'], state['vG'], state['wG'] #Translation Velocities from Current State
    pP, qP, rP = state['pP'], state['qP'], state['rP'] #Angular Velocities of Parafoil
    pC, qC, rC = state['pC'], state['qC'], state['rC'] #Angular Velocities of Cradle

    #Getting Controls
    deltaL = control.get('delta_left', 0.0)
    deltaR = control.get('delta_right', 0.0)
    
    




        