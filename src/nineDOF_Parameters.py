#nineDOF_Parameters.py
import numpy as np

#Parafoil Parameters
m_parafoil = 8.99 #Parafoil Mass [kg]
A_parafoil = 27 #Parafoil Area [m^2]
c_bar_parafoil = 3.87 #[m]
b_bar_parafoil = 6.99 #[m]
d_bar_parafoil = 1.0 
deadband_parafoil = 0.0
I_parafoil = ([[74.56, 0,      0],
              [0,       14.62, 0],
              [0,       0,      82.8]]) #[kg*m^2]


#Cradle Parameters
m_cradle = 90.0 #Cradle Mass [kg]
A_cradle = 0.4337 #Cradle Area [m^2]
C_D_cradle = 1.0 #Cradle Drag Coefficient
I_cradle = ([[9.378, 0,      0],
            [0,     6.0518, 0],
            [0,     0,      6.2401]]) #[kg*m^2]

#Gimbal Parameters
K_G = 25 #Gimbal Rotational Stiffness [N-m/rad]
C_G = 10 #Gimbal Rotational Damping [N-m-s/rad]

#System Geometric Parameters
x_gc = 0.0 #X Distance from Gimbal to Cradle Center of Gravity [m]
y_gc = 0.0 #Y Distance from Gimbal to Cradle Center of Gravity [m]
z_gc = 0.47 #Z Distance from Gimbal to Cradle Center of Gravity [m]

x_gp = 0.0 #X Distance from Cradle to Parafoil Center of Gravity [m]
y_gp = 0.0 #Y Distance from Cradle to Parafoil Center of Gravity [m]
z_gp = -5.75 #Z Distance from Cradle to Parafoil Center of Gravity [m]

x_pr = 0.0 #X Distance from Parafoil Center of Gravity to Rotation Point [m]
y_pr = 0.0 #Y Distance from Parafoil Center of Gravity to Rotation Point [m]
z_pr = 0.0 #Z Distance from Parafoil Center of Gravity to Rotation Point [m]

x_rap = 0.0 #X Distance from Rotation Point to Aerodynamic Center [m]
y_rap = 0.0 #Y Distance from Rotation Point to Aerodynamic Center [m]
z_rap = 0.0 #Z Distance from Rotation Point to Aerodynamic Center [m]

x_pmp = 0.0 #X Distance from Parafoil Center of Gravity to Apparent Mass Center [m]
y_pmp = 0.0 #Y Distance from Parafoil Center of Gravity to Apparent Mass Center [m]
z_pmp = 5.75 #Z Distance from Parafoil Center of Gravity to Apparent Mass Center [m]

#Basic Apparent Mass Matrix
I_AM = ([[0.984, 0,      0],
         [0,     0.0988, 0],
         [0,     0,      36.405]]) #[kg]

#Basic Apparent Inertia Matrix
I_AI = ([[194.21, 0,     0],
         [0,      7.531, 0],
         [0,      0,     6.9106]]) #[kg-m]

#Spanwise Camber Matrix
I_H = ([[0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]]) #[kg-m]

gamma_nom = 0.0610 #Nominal Incidence Angle [rad]
delta_nom = 0.3 #Nominal Symmetric Break Deflection
tau = 2.0 #Brake Deflection Time Constant (Note actuators are also rate limited in code)

#Aerodymamic Parameters
CM0 = 0.0
CMQ = -1.00
CMDS = 0.00
CYB = -1.00
CNB = 0.0 #(-1.00) CHECK IF ERRORS PERSIST
CLB = 0.0
CLP = 0.112
CLR = 0.253
CLDA = 0.0000
CNP = -0.050
CNR = -0.212
CND1 = 0.000
        #       Sig      CD0    CDA2     CL0     CLA
aeroTable = ([[0.2500, 0.1373, 0.0977, -0.2636, 2.8139],
              [1.5000, 0.0351, 3.9642, 0.1313, 1.9825]])
        #      AOA    CNDA2
AOATable = ([[0.1800, 0.0280], 
             [0.2500, 0.0350], 
             [0.3200, 0.0700]])

def getInterpolatedAero(deltaS, alpha):
    """
    Given δS/ d̄ and AOA, return (CD0, CDA2, CL0, CLA, CNDA2) via linear interpolation.
    If asked value is outside the tabulated range, this clamps to the ends.
    """
    x0 = aeroTable[:,0]
    x1 = AOATable[:,0]

    CD0 = np.interp(deltaS, x0, aeroTable[:,1])
    CDA2 = np.interp(deltaS, x0, aeroTable[:,2])
    CL0 = np.interp(deltaS, x0, aeroTable[:,3])
    CLA = np.interp(deltaS, x0, aeroTable[:,4])
    CND2 = np.interp(alpha, x1, AOATable[:,1])

    return CD0, CDA2, CL0, CLA, CND2



