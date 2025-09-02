#nineDOF_Parameters.py

#Parafoil Parameters
m_parafoil = 3.7 #Parafoil Mass [kg]
A_parafoil = 18.5 #Parafoil Area [m^2]
c_bar_parafoil = 2.1 #[m]
b_bar_parafoil = 8.8 #[m]
d_bar_parafoil = 1.0 
deadband_parafoil = 0.0
I_parafoil = ([[45.5273, 0,      0],
              [0,       9.6531, 0],
              [0,       0,      45.8616]]) #[kg*m^2]


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

#Panel Parameters
num_panels = 9 #Total Number of Panels
A_panel = [0.925, 2.6429, 2.6429, 2.6429, 2.6429, 2.6429, 2.6429 ,2.6429, 0.925] #Area of Panels (Ordered Left to Right: 1, 2, 3, 4, 5, 6, 7, 8, 9) [m^2]

SL_panel = [-2.1023, 0, 0, 0, 0, 0, 0, 0, -2.1023] #Stationline from Center of Gravity (Ordered Left to Right: 1, 2, 3, 4, 5, 6, 7, 8, 9) [m]
BL_panel = [-4.4, -4.4, -2.9333, -1.4667, 0, 1.4667, 2.9333, 4.4, 4.4] #Buttline from Center of Gravity (Ordered Left to Right: 1, 2, 3, 4, 5, 6, 7, 8, 9) [m]
WL_panel = [0, 0, -1.2536, -1.5856, -1.6818, -1.5856, -1.2536, 0, 0] #Waterline from Center of Gravity (Ordered Left to Right: 1, 2, 3, 4, 5, 6, 7, 8, 9) [m]

phi_panel = [-1.5708, -0.7854, -0.5236, -0.2618, 0, 0.2618, 0.5236, 0.7854, 1.5708] #Phi Angle [rad]
theta_panel = [0, 0, 0, 0, 0, 0, 0, 0, 0] #Theta Angle [rad]

CL0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
CLDELTA = [0.0, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.0] 
CLDELTA3 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
CLA = [3.9927, 5.2029, 5.2029, 5.2029, 5.2029, 5.2029, 5.2029, 5.2029, 3.9927]
CD0 = [0.018, 0.018, 0.018, 0.018, 0.018, 0.018, 0.018, 0.018, 0.018]
CDA2 = [1.6891, 1.6891, 1.6891, 1.6891, 1.6891, 1.6891, 1.6891, 1.6891, 1.6891]
CDDELTA = [0.0, 0.063949, 0.063949, 0.063949, 0.063949, 0.063949, 0.063949, 0.063949, 0.0]
CDDELTA3 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

