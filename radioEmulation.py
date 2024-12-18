from PID_config import *

dt = 1/100          #100Hz loop

'''
Get PID coeffs?

MSP_PID  112    //out message    P I D coeff (9 are used currently)
'''

'''
Raw PID -> clamp Raw PID -> map it onto the pwm value 
'''


#returns pid, previous error based on the given params 
def get_pid(current_state, desired_state, previous_error, P, I, D):
    current_error = desired_state - current_state
    dErr = current_error - previous_error
    p_term = P * current_error
    i_term = I * dErr * dt                      #simple trapezoid rule
    d_term = D * dErr / dt
    pid = p_term + i_term + d_term
    return pid, current_error                   #current error becomes previous


def get_throttle_pid():
    pass

def get_yaw_pid():
    pass

def get_pitch_pid():
    pass

def get_roll_pid():
    pass





