#nineDOF_Main.py

state0 = {
    "xG": 0.0, "yG": 0.0, "zG": 100.0, #X, Y, and Z of Gimbal
    "uG": 0.0, "vG": 0.0, "wG": 0.0, #U, V, and W of Gimbal 
    "pP": 0.0, "qP": 0.0, "rP":0.0, #P, Q, and R of Parafoil
    "pC": 0.0, "qC": 0.0, "rC": 0.0, #P, Q, and R of Cradle
    "p_quat": [0, 0, 0, 1], #Parafoil Angular Position Quaternion
    "c_quat": [0, 0, 0, 1] #Cradle Angular Position Quaternion
}

statedot0 = {
    "xG_dot" : 0.0, "yG_dot": 0.0, "zG_dot": 0.0, #Xdot, Ydot, and Zdot of Gimbal
    "uG_dot" : 0.0, "vG_dot" : 0.0, "wG_dot" : 0.0, #Udot, Vdot, and Wdot of Gimbal
    "pP_dot" : 0.0, "qP_dot" : 0.0, "rP_dot" : 0.0, #Pdot, Qdot, and Rdot of Parafoil
    "pC_dot" : 0.0, "rC_dot" : 0.0, "qC_dot" : 0.0, #Pdot, Qdot, and Rdot of Cradle
    "p_phi_dot" : 0.0, "p_theta_dot" : 0, "p_psi_dot" : 0.0, #PHIdot, THETAdot, and Psidot of Parafoil
    "c_phi_dot" : 0.0, "c_theta_dot" : 0, "c_psi_dot" : 0.0, #PHIdot, THETAdot, and Psidot of Parafoil
}