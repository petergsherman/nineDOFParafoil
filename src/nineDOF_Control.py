#nineDOF_Control.py
class BaseController:
    def compute_control(self, state):
        return {
            "delta_left": 0.0,
            "delta_right": 0.0
        }
        
# ---------------------------------------
# Controller Factory
# ---------------------------------------

def get_controller(name, **kwargs):
    name = name.lower()
    if name == "basecontroller":
        return BaseController()