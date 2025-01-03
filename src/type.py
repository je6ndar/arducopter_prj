import numpy as np
from collections import deque


class RC_CHANNELS:
    def __init__(self):
        self.rc1 = int(0)
        self.rc2 = int(0)
        self.rc3 = int(0)
        self.rc4 = int(0)
        self.rc5 = int(0)
        self.rc6 = int(0)
        self.rc7 = int(0)
        self.rc8 = int(0)
        self.rc9 = int(0)
        self.rc10 = int(0)
        self.rc11 = int(0)
        self.rc12 = int(0)
        self.rc13 = int(0)
        self.rc14 = int(0)
        self.rc15 = int(0)
        self.rc16 = int(0)
        self.rc17 = int(0)
        self.rc18 = int(0)

    def update_controll_channels(self, channels):
        #sitl mapping
        self.rc1 = int(channels['roll'])
        self.rc2 = int(channels['pitch'])
        self.rc4 = int(channels['yaw'])
        self.rc3 = int(channels['throttle'])

    def get_rc_vec(self):
        vec = list(vars(self).values())
        return np.array(vec) 
    def get_rpyt_vec(self):
        vec = [self.rc1, self.rc2, self.rc4, self.rc3]
        return np.array(vec)

class IMU:
    def __init__(self):
        self.ax = None
        self.ay = None
        self.az = None
    def update(self, acc):
        self.ax = acc[0]
        self.ay = acc[1]
        self.az = acc[2]
    def scale(self, scale_factor):
        self.ax*=scale_factor
        self.ay*=scale_factor
        self.az*=scale_factor
    def get_acc_vec(self):
        vec = list(vars(self).values())
        return np.array(vec)
    def get_status(self):
        return all(value is not None for value in vars(self).values())
    def clear(self):
        self.__init__()
    
class CurrentState:
    def __init__(self, state=None):
        if state:
            self.roll = state.get('roll')
            self.pitch = state.get('pitch')
            self.yaw = state.get('yaw')
            self.alt = state.get('alt')
            #self.time = state.get('time', None)
        else:
            self.roll = None
            self.pitch = None
            self.yaw = None
            self.alt = None
           #self.time = None

    # def update(self, state):
    #     if state['roll']:
    #         self.roll = state['roll']
    #     if state['pitch']:
    #         self.pitch = state['pitch']
    #     if state['yaw']:
    #         self.yaw = state['yaw']
    #     if state['alt']:
    #         self.alt = state['alt']        
    
    def get_status(self):
        return all(value is not None for value in vars(self).values())
    
    def clear(self):
        #print("state clear call")
        self.__init__()
    
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