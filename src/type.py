import numpy as np

class RC_CHANNELS:
    def _init_(self,channels):
        self.RC = channels

class CurrentState:
    def _init_(self, state):
        self.roll = state['roll']
        self.pitch = state['pitch']
        self.yaw = state['yaw']
        self.alt = state['alt']
    def get_current_state():
        pass
    

class DesiredState:
    def _init_(self):
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.alt = None
    def clear(self):
        self._init_()

    def get_desired_state():
        pass