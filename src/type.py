import numpy as np
from collections import deque


class RC_CHANNELS:
    def __init__(self):
        self.rc1 = 0
        self.rc2 = 0
        self.rc3 = 0
        self.rc4 = 0
        self.rc5 = 0
        self.rc6 = 0
        self.rc7 = 0
        self.rc8 = 0
        self.rc9 = 0
        self.rc10 = 0
        self.rc11 = 0
        self.rc12 = 0
        self.rc13 = 0
        self.rc14 = 0
        self.rc15 = 0
        self.rc16 = 0
        self.rc17 = 0
        self.rc18 = 0

    def update_controll_channels(self, channels):
        self.rc1 = channels['roll']
        self.rc3 = channels['pitch']
        self.rc4 = channels['yaw']
        self.rc2 = channels['throttle']

    def get_rc_vec(self):
        vec = list(vars(self).values())
        return np.array(vec) 

class IMU:
    def __init__(self):
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.ax_LPfiltered = 0
        self.ay_LPfiltered = 0
        self.az_LPfiltered = 0
    def update(self, acc):
        self.ax = acc[0]
        self.ay = acc[1]
        self.az = acc[2]
    def scale(self, scale_factor):
        self.ax*=scale_factor
        self.ay*=scale_factor
        self.az*=scale_factor
    def run_LPfilter(self, alpha):
        self.ax_LPfiltered = alpha*self.ax_LPfiltered + (1-alpha)*self.ax
        self.ay_LPfiltered = alpha*self.ay_LPfiltered + (1-alpha)*self.ay
        self.az_LPfiltered = alpha*self.az_LPfiltered + (1-alpha)*self.az
    def get_acc_filtered(self):
        return np.array([self.ax_LPfiltered, self.ay_LPfiltered, self.az_LPfiltered])
class CurrentState:
    def __init__(self, state=None):
        if state:
            self.roll = state.get('roll', None)
            self.pitch = state.get('pitch', None)
            self.yaw = state.get('yaw', None)
            self.alt = state.get('alt', None)
        else:
            self.roll = None
            self.pitch = None
            self.yaw = None
            self.alt = None
    
    def update(self, state):
        if state['roll'] is not None:
            self.roll = state['roll']
        if state['pitch'] is not None:
            self.pitch = state['pitch']
        if state['yaw'] is not None:
            self.yaw = state['yaw']
        if state['alt'] is not None:
            self.alt = state['alt']        
    
    def get_status(self):
        return all(value is not None for value in vars(self).values())
    
    def clear(self):
        self._init_()
    
    def get_state_vec(self):
        vec = list(vars(self).values())
        return np.array(vec)

    def get_missing(self):
        return vars(self)
    def get_current_state():
        pass
    

class DesiredState(CurrentState):
    def set_initial_condition(self, current_state):
        self.yaw = current_state.yaw
        self.alt = current_state.alt
    def update_desired_roll_pitch(self, roll, pitch):
        self.roll = roll
        self.pitch = pitch

# Circular buffer for 3D accelerometer data
class CircularBuffer3D:
    def __init__(self, size):
        self.buffer_x = deque(maxlen=size)
        self.buffer_y = deque(maxlen=size)
        self.buffer_z = deque(maxlen=size)

    def add(self, accel):
        # accel is a 3D vector (ax, ay, az)
        self.buffer_x.append(accel[0])
        self.buffer_y.append(accel[1])
        self.buffer_z.append(accel[2])

    def is_full(self):
        return len(self.buffer_x) == self.buffer_x.maxlen

    def get_data(self):
        return np.array([self.buffer_x, self.buffer_y, self.buffer_z])