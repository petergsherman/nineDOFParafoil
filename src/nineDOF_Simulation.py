#nineDOF_Simulation.py
from nineDOF_Dynamics import compute_dynamics
from nineDOF_Control import get_controller

def rk4_step (state, dt, controller):

    def state_derivatives(s):
        control = controller.compute_control(s)

        statedot = compute_dynamics(s, control)

        return statedot
    
    s1 = state_derivatives(state)
    s2 = state_derivatives(state_step(state, s1, dt / 2))
    s3 = state_derivatives(state_step(state, s2, dt / 2))
    s4 = state_derivatives(state_step(state, s3, dt))

    new_state = []
    for i in state:
        new_state = state[i] + ((dt / 6) * (s1[i] + 2*s2[i] + 2*s3[i] + s4[i]))
    
    return new_state

def state_step(state, delta, factor):
    step_state = []

    for i in state:
        step_state[i] = state[i] + (factor * delta[i])

    return step_state

def run_simulation(state0, dt = 0.001, z_stop = 0.0, controller_type = None, controller_kwargs = None):
    controller_kwargs = controller_kwargs or {}
    controller = get_controller(name = controller_type, **controller_kwargs)

    state = state0.copy
    simulationData = [state.copy()]

    while state[2] > z_stop:
        state = rk4_step(state, dt, controller)
        simulationData.append(state.copy())
    
    return simulationData