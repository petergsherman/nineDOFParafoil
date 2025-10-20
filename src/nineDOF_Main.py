#nineDOF_Main.py
import numpy as np
from nineDOF_Simulation import run_simulation
from nineDOF_Visualization import plot_trajectory_3D

state0 = np.array([0, 0, 25, #xG, yG, zG
                   0, 0, 0, #phiP, thetaP, psiP
                   0, 0, 0, #phiC, thetaC, psiC
                   6, 0, 0, #uG, vG, wG
                   0, 0, 0, #pP, qP, rP
                   0, 0, 0])#pC, qC, rC

statedot0 = np.array([0, 0, 0, #xGdot, yGdot, zGdot
                      0, 0, 0, #phiPdot, thetaPdot, psiPdot
                      0, 0, 0, #phiCdot, thetaCdot, psiCdot
                      0, 0, 0, #uGdot, vGdot, wGdot
                      0, 0, 0, #pPdot, qPdot, rPdot
                      0, 0, 0])#pCdot, qCdot, rCdot

data = run_simulation(state0, 0.001, 0, "basecontroller", None)
plot_trajectory_3D(data)

