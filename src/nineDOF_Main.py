#nineDOF_Main.py
import numpy as np

state0 = np.array([0, 0, 0, #xG, yG, zG
                   0, 0, 0, #phiP, thetaP, psiP
                   0, 0, 0, #phiC, thetaC, psiC
                   0, 0, 0, #uG, vG, wG
                   0, 0, 0, #pP, qP, rP
                   0, 0, 0])#pC, qC, rC

statedot0 = np.array([0, 0, 0, #xGdot, yGdot, zGdot
                      0, 0, 0, #phiPdot, thetaPdot, psiPdot
                      0, 0, 0, #phiCdot, thetaCdot, psiCdot
                      0, 0, 0, #uGdot, vGdot, wGdot
                      0, 0, 0, #pPdot, qPdot, rPdot
                      0, 0, 0])#pCdot, qCdot, rCdot


