
class GamepadProfile:

    axis_left_x = 0
    axis_left_y = 1
    axis_right_x = 2
    axis_right_y = 3
    axis_brake = 5
    axis_gas = 4
    btn_A = 0
    btn_B = 1
    btn_X = 3
    btn_Y = 4
    btn_LB = 6
    btn_RB = 7
    btn_VIEW = 10
    btn_MENU = 11
    btn_HOME = 12
    btn_LSB = 13
    btn_RSB = 14
    hat_dpad = 0

    # # Axes mapping
    # axis_left_x = 0
    # axis_left_y = 1
    # axis_right_x = 2
    # axis_right_y = 3
    # axis_brake = 4
    # axis_gas = 5

    # # Buttons mapping
    # btn_A = 0
    # btn_B = 1
    # btn_X = 2
    # btn_Y = 3
    # btn_LB = 4
    # btn_RB = 5
    # btn_VIEW = 6 # Back / Select
    # btn_MENU = 7 # Start
    # btn_HOME = 8
    # btn_LSB = 9  # Left stick click
    # btn_RSB = 10 # Right stick click
    
    # # Hats mapping
    # hat_dpad = 0
    
    @classmethod
    def normalize_axes(cls, raw_axes, raw_hats):
        # Default normalization for standard axes (-1 to 1) and triggers (often -1 to 1 but we want 0 to 1)
        axes_state = {
            "left_stick": [0.0, 0.0],
            "right_stick": [0.0, 0.0],
            "brake": 0.0,
            "gas": 0.0,
            "dpad": [0, 0]
        }
        
        if len(raw_axes) > cls.axis_left_x: axes_state["left_stick"][0] = raw_axes[cls.axis_left_x]
        if len(raw_axes) > cls.axis_left_y: axes_state["left_stick"][1] = raw_axes[cls.axis_left_y]
        if len(raw_axes) > cls.axis_right_x: axes_state["right_stick"][0] = raw_axes[cls.axis_right_x]
        if len(raw_axes) > cls.axis_right_y: axes_state["right_stick"][1] = raw_axes[cls.axis_right_y]
        
        # Triggers often go from -1 (unpressed) to 1 (fully pressed) in pygame
        if len(raw_axes) > cls.axis_brake: axes_state["brake"] = (raw_axes[cls.axis_brake] + 1) / 2.0
        if len(raw_axes) > cls.axis_gas: axes_state["gas"] = (raw_axes[cls.axis_gas] + 1) / 2.0
        
        if len(raw_hats) > cls.hat_dpad:
            axes_state["dpad"] = list(raw_hats[cls.hat_dpad])
            
        return axes_state

    @classmethod
    def map_buttons(cls, raw_buttons):
        # Creates a dictionary of named buttons
        state = {}
        for attr in dir(cls):
            if attr.startswith("btn_"):
                idx = getattr(cls, attr)
                name = attr[4:] # strip btn_
                if idx is not None and idx < len(raw_buttons):
                    state[name] = bool(raw_buttons[idx])
        return state

    @classmethod
    def get_default_button_states(cls):
        state = {}
        for attr in dir(cls):
            if attr.startswith("btn_"):
                name = attr[4:]
                state[name] = "up"
        return state
