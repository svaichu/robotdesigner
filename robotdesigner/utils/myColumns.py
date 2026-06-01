
# from ray.rllib.core.columns import Columns

class myColumns():
    T = "timestep"

    STATE = "state"

    ACTIONS = "action"
    LAST_ACTION = "last_action"

    TARGET = "target"
    CURR = "curr"

    # zero spawn plus noise
    LAST_RESET_STATE = "reset_state"