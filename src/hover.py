import numpy as np
import queue

import config as config
import mavlink as mavlink
import type as types
import state as state


'''
Raw PID -> clamp Raw PID -> map it onto the pwm value 
'''

'''
Position estimate:

Euler angles -> rotational matrix -> rot matrix@
'''
dt = 0.01          #100Hz loop

#INITIAL_POSITION = np.array([0,0])   #we set this as initial coordinates in NED

DESIRED_STATE = types.DesiredState()  #ALL NONE


def hover(CurrentAttitudeQueue=None):
    global DESIRED_STATE

    if not CurrentAttitudeQueue:
        return
    try:
        current_state = CurrentAttitudeQueue.get_nowait()
        CurrentAttitudeQueue.task_done()
    except queue.Empty as error:
        if state.STATE==state.IDLE:
            DESIRED_STATE.clear() #Sets all to None 
    
    xy = get_coordinate_estimate(current_state)
    
    position_contoller(current_state)



# Returns estimated coordinates in NED based on the measurements from the drone
def get_coordinate_estimate(current_state):
    #should we run kalman filter in there?
    #acceleration estimation -> double integration Runge-Kutta
    #return [x, y]
    pass


# Returns desired roll and pitch 
def position_contoller(current_state, xy):  #current_state for yaw, xy - estimated coordinate in NED
    # 1. World to body conversion
    # 2. Desired Pitch and roll
    xy_err = -xy                        #as we set desired xy to be [0,0]
    angle = current_state.yaw           #need to check in what format we get yaw
    
    dcm = dcm(angle)

    pass



def get_throttle_rc():
    pass

def get_yaw_rc():
    pass

def get_pitch_rc():
    pass

def get_roll_rc():
    pass