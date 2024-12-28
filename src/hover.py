import numpy as np
import queue

import config as config
import mavlink as mavlink
import type as types
import state as state
import helpers as helpers

'''
Raw PID -> clamp Raw PID -> map it onto the pwm value 
'''

'''
Position estimate:

Euler angles -> rotational matrix -> rot matrix@
'''
dt = 0.01          #100Hz loop
scale_imu = 0.00981
#INITIAL_POSITION = np.array([0,0])   #we set this as initial coordinates in NED

desired_state = types.DesiredState()  #ALL NONE

# Initial conditions (3D)
initial_velocity = np.array([0, 0, 0])  # Starting from rest
initial_displacement = np.array([0, 0, 0])  # Starting at origin
BATCH_SIZE = 4
accel_buffer = types.CircularBuffer(BATCH_SIZE)

def hover(CurrentAttitudeQueue=None):
    global desired_state, accel_buffer

    if not CurrentAttitudeQueue:
        return
    while True:
        try:
            vehicle_data = CurrentAttitudeQueue.get_nowait()
            current_state = vehicle_data['attitude']
            imu = vehicle_data['IMU']
            # If no data is missing in current_state and there is data missing in desired_state
            if current_state.get_status() and not desired_state.set_status():    
                desired_state.set_initial_cond(current_state)       # Update desired_state with current_state
            # If data is missing in both states, skip this iteration
            elif not current_state.get_status() and not desired_state.set_status():   
                continue
            CurrentAttitudeQueue.task_done()
        except queue.Empty as error:
            if state.STATE==state.IDLE:
                desired_state.clear() #Sets all to None 
        imu.scale(scale_imu)
        imu.run_LPfilter(0.6)  #scale factor
        Acc = imu.get_acc_filtered()
        accel_buffer.add(Acc)
        if accel_buffer.is_full():
            xy = get_coordinate_estimate(current_state, Acc)
            roll, pitch = position_contoller(current_state, xy)
        desired_state.update_desired_roll_pitch(roll, pitch)   #we have complete desired state at this point 
    
        #Now get difference between desired and actual state and run pid loops
        current_state_vec = current_state.get_state_vec()
        desired_state_vec = desired_state.get_state_vec()

        error = current_state_vec - desired_state_vec   #roll, pitch, yaw, alt 

        throttle_pid = helpers.get_pid(error[3], config.throttle_P, config.throttle_I, config.throttle_D, dt)
        yaw_pid = helpers.get_pid(error[1], config.yaw_P, config.yaw_I, config.yaw_D, dt)
        roll_pid =helpers.get_pid(error[0], config.roll_P, config.roll_I, config.roll_D, dt)
        pitch_pid = helpers.get_pid(error[2], config.pitch_P, config.pitch_I, config.pitch_D, dt)




# Returns estimated coordinates in NED based on the measurements from the drone
def get_coordinate_estimate(current_state, Acc):
    #https://www.youtube.com/watch?v=T9jXoG0QYIA&list=PLqinEaadXCHY1O8ZVoq_Uyjpp3zvvBe66&ab_channel=ScottLobdell
    #should we run kalman filter in there?
    #acceleration estimation -> double integration Runge-Kutta
    #return [x, y]
    rpy = current_state.get_state_vec()[:-1]
    dcm = helpers.euler_to_dcm(rpy)   #input [roll, pitch, yaw] 
    dcm = np.array(dcm)
    Acc_NED = dcm.T@Acc - np.array([0,0,9.81])  #dcm^-1 = dcm.T as dcm is orthonormal
    
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