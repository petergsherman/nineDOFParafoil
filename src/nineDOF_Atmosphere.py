#nineDOF_Atmosphere.py
import numpy as np 

def getAirDensity(altitude):

    T = 0.0
    p = 0.0
    rho = 0.0

    if altitude > 25000: #(Upper Stratosphere)
        T = -131.21 + 0.002999 * altitude
        p = 2.488 * ((T + 273.1) / 216.6)**(-11.388)
    elif 11000 < altitude < 25000:#(Lower Stratosphere)
        T = -56.46
        p = 22.65 * np.exp(1.73 - (0.000157 * altitude))
    elif altitude < 11000: #(Troposphere)
        T = 15.04 - 0.00649 * altitude
        p = 101.29 * ((T + 273.1) / 288.08)**(5.256)
    else:
        rho = 1.224

    rho = p / (0.2869 * (T + 273.1))

    return rho

def getWindVector(altitude):
    uW, vW, wW = 0.0, 0.0, 0.0
    