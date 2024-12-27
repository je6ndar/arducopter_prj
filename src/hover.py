import numpy as np
import queue

import config as config
import mavlink as mavlink
import type as types
import state as state


'''
Get PID coeffs?

MSP_PID  112    //out message    P I D coeff (9 are used currently)
'''

'''
Raw PID -> clamp Raw PID -> map it onto the pwm value 
'''

'''
Position estimate:

Euler angles -> rotational matrix -> rot matrix@
'''
dt = 0.01          #100Hz loop

INITIAL_POSITION = np.array([0,0])


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
    
    position_contoller(current_state)

#returns pid, previous error based on the given params 
def get_pid(current_state, desired_state, previous_error, P, I, D):
    current_error = desired_state - current_state
    dErr = current_error - previous_error
    p_term = P * current_error
    i_term = I * dErr * dt                      #simple trapezoid rule
    d_term = D * dErr / dt
    pid = p_term + i_term + d_term
    return pid, current_error                   #current error becomes previous

# Returns estimated state based on the measurements from the drone
def get_state():
    #should we run kalman filter in there?  
    pass


# Returns desired roll and pitch 
def position_contoller(current_state):
    # 1. World to body conversion
    # 2. Desired Pitch and roll
    angle = CURRENT_POSITION_ESTIMATE['yaw'] # need to check in what format we get yaw
    coordinate = CURRENT_POSITION_ESTIMATE['coordinate']
    dcm = dcm(angle)
    local 
    pass

# Returns 2x2 rotational matrix
def dcm(angle):
    dcm = np.zeros([2,2])
    dcm[0,0] = np.cos(angle)
    dcm[0,1] = -np.sin(angle)
    dcm[1,0] = np.sin(angle)
    dcm[1,1] = np.cos(angle)
    return dcm






def get_throttle_pid():
    pass

def get_yaw_pid():
    pass

def get_pitch_pid():
    pass

def get_roll_pid():
    pass