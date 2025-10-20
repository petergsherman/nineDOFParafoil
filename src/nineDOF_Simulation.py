# nineDOF_Simulation.py
from nineDOF_Dynamics import compute_dynamics
from nineDOF_Control import get_controller
import numpy as np

def rk4_step(state, dt, controller):
    def f(s):
        control = controller.compute_control(s)
        return compute_dynamics(s, control)

    s1 = f(state)
    s2 = f(state + 0.5*dt*s1)
    s3 = f(state + 0.5*dt*s2)
    s4 = f(state + dt*s3)
    return state + (dt/6.0)*(s1 + 2*s2 + 2*s3 + s4)

def run_simulation(state0, dt=0.001, z_stop=0.0, controller_type="basecontroller", controller_kwargs=None):
    controller_kwargs = controller_kwargs or {}
    controller = get_controller(name=controller_type, **controller_kwargs)

    state = state0.copy()         # <-- needs () to copy
    simulationData = [state.copy()]

    while state[2] > z_stop:
        state = rk4_step(state, dt, controller)
        simulationData.append(state.copy())
        print('z = ', state[2], 'u = ', state[9])

    return simulationData
