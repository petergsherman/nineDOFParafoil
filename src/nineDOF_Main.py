#nineDOF_Main.py

state0 = {
    "xG": 0.0, "yG": 0.0, "zG": 100.0, #X, Y, and Z of Gimbal
    "uG": 0.0, "vG": 0.0, "wG": 0.0, #U, V, and W of Gimbal 
    "pP": 0.0, "qP": 0.0, "rP":0.0, #P, Q, and R of Parafoil
    "pC": 0.0, "qC": 0.0, "rC": 0.0, #P, Q, and R of Cradle
    "p_quat": [0, 0, 0, 1], #Parafoil Angular Position Quaternion
    "c_quat": [0, 0, 0, 1] #Cradle Angular Position Quaternion
}