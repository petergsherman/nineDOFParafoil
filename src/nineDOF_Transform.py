#nineDOF_Transform.py
import numpy as np

def skew(v):
    vx, vy, vz = v
    return np.array([[0, -vz, vy],
                     [vz, 0, -vx],
                     [-vy, vx, 0]])

def makeT_IP(euler): #Transformation from Inertial Frame to Parafoil Frame
    phi, theta, psi = euler
    c_phi = np.cos(phi)
    c_theta = np.cos(theta)
    c_psi = np.cos(psi)

    s_phi = np.sin(phi)
    s_theta = np.sin(theta)
    s_psi = np.sin(psi)

    return np.array([[c_theta * c_psi,    s_phi * s_theta * c_psi - c_phi * s_psi,    c_phi * s_theta * c_psi + s_phi * s_psi],
                     [c_theta * s_psi,    s_phi * s_theta * s_psi + c_phi * c_psi,    c_phi * s_theta * s_psi - s_phi * c_psi],
                     [s_theta,            s_phi * c_theta,                            c_phi * c_theta]])

def makeT_IC(euler): #Transformation from Inertial Frame to Cradle Frame
    phi, theta, psi = euler
    c_phi = np.cos(phi)
    c_theta = np.cos(theta)
    c_psi = np.cos(psi)

    s_phi = np.sin(phi)
    s_theta = np.sin(theta)
    s_psi = np.sin(psi)

    return np.array([[c_theta * c_psi,    s_phi * s_theta * c_psi - c_phi * s_psi,    c_phi * s_theta * c_psi + s_phi * s_psi],
                     [c_theta * s_psi,    s_phi * s_theta * s_psi + c_phi * c_psi,    c_phi * s_theta * s_psi - s_phi * c_psi],
                     [-s_theta,           s_phi * c_theta,                            c_phi * c_theta]])

def makeH(euler):
    phi, theta, psi = euler
    c_phi = np.cos(phi)
    c_theta = np.cos(theta)
    s_phi = np.sin(phi)
    t_theta = np.tan(theta)

    return np.array([[1,  s_phi * t_theta,     c_phi * t_theta],
                     [0,  c_phi,               -s_phi],
                     [0,  s_phi / c_theta,     c_phi / c_theta]])

def makeT_PPI (incidence, nomincidence): # Rotation by Parafoil Incidence Angle 
    c_incidence = np.cos(incidence + nomincidence)
    s_incidence = np.sin(incidence + nomincidence)

    return np.array([[c_incidence,  0,  s_incidence], 
                     [0,            1,  0], 
                     [-s_incidence, 0 , c_incidence]])

