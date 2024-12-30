import numpy as np
import queue
#from scipy.integrate import trapezoid

import src.config as config
import src.mavlink as mavlink
import src.type as types
import src.state as state
import src.helpers as helpers

'''
Raw PID -> clamp Raw PID -> map it onto the pwm value 
'''

'''
Position estimate:

Euler angles -> rotational matrix -> rot matrix@
'''
dt = 0.01          #100Hz loop
t_last = None
scale_imu = 0.00981
#INITIAL_POSITION = np.array([0,0])   #we set this as initial coordinates in NED

desired_state = types.DesiredState()  #ALL NONE
rc_channels = types.RC_CHANNELS()

#BATCH_SIZE = 4
#accel_buffer = types.CircularBuffer(BATCH_SIZE)
previous_state_error = np.zeros(4)

def hover(CurrentAttitudeQueue=None, MavlinkSendQueue=None):
    global desired_state, previous_state_error, rc_channels, dt, t_last

    if not CurrentAttitudeQueue:
        return
    while True:
        try:
            print("+++++++++++++++++++HOVER+++++++++++++++++++++++++++++++")
            vehicle_data = CurrentAttitudeQueue.get()
            current_state = vehicle_data['attitude']
            imu = vehicle_data['IMU']
            if t_last:
                dt = (vehicle_data['time']- t_last)/1000
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
                zero_all()  #function that zeros all stored data related to state estimation and errors
        imu.scale(scale_imu)
        imu.run_LPfilter(0.8)  #scale factor
        Acc = imu.get_acc_filtered()
        ''' 
        accel_buffer.add(Acc)
        if accel_buffer.is_full():
            xy = get_coordinate_estimate(current_state, accel_buffer)
            roll, pitch = position_contoller(current_state, xy)
        '''
        xy = get_coordinate_estimate(current_state, Acc)
        desired_roll, desired_pitch = position_contoller(xy)
        desired_state.update_desired_roll_pitch(desired_roll, desired_pitch)   #we have complete desired state at this point 
    
        #Now get difference between desired and actual state and run pid loops
        current_state_vec = current_state.get_state_vec()
        desired_state_vec = desired_state.get_state_vec()

        current_state_error = current_state_vec - desired_state_vec   #roll, pitch, yaw, alt 

        throttle_pid = helpers.get_pid(current_state_error[3], previous_state_error[3], config.throttle_P, config.throttle_I, config.throttle_D, dt)
        yaw_pid = helpers.get_pid(current_state_error[1], previous_state_error[1], config.yaw_P, config.yaw_I, config.yaw_D, dt)
        roll_pid =helpers.get_pid(current_state_error[0], previous_state_error[0], config.roll_P, config.roll_I, config.roll_D, dt)
        pitch_pid = helpers.get_pid(current_state_error[2], previous_state_error[2], config.pitch_P, config.pitch_I, config.pitch_D, dt)

        previous_state_error = current_state_error

        throttle_rc = get_throttle_rc(throttle_pid)
        yaw_rc = get_yaw_rc(yaw_pid)
        roll_rc = get_roll_rc(roll_pid)
        pitch_rc = get_pitch_rc(pitch_pid)
        print("=========================================================")
        #pack controll channels into dictionary and update 
        controll_channels = controll_channels_pack(roll=roll_rc, pitch=pitch_rc, yaw=yaw_rc, throttle=throttle_rc)
        rc_channels.update_controll_channels(controll_channels)

        MavlinkSendQueue.put(rc_channels)


def controll_channels_pack(roll, pitch, yaw, throttle):
    return {'roll': roll, 'pitch': pitch, 'yaw': yaw, 'throttle': throttle}

acc_stored = np.zeros(2) #previous values of acceleration in NED
vel_stored = np.zeros(2) #previous values of velocity in NED
xy_stored = np.zeros(2) #previous values of position in NED
# Returns estimated coordinates in NED based on the measurements from the drone
def get_coordinate_estimate(current_state, Acc):
    global acc_stored, vel_stored, xy_stored
    #https://www.youtube.com/watch?v=T9jXoG0QYIA&list=PLqinEaadXCHY1O8ZVoq_Uyjpp3zvvBe66&ab_channel=ScottLobdell
    #should we run kalman filter in there?
    #acceleration estimation -> double integration Runge-Kutta !!! simple trapezoid for now. 
    rpy = current_state.get_state_vec()[:-1]
    dcm = helpers.dcm(rpy)   #input [roll, pitch, yaw] 
    dcm = np.array(dcm)
    #3d vector of acceleration in NED frame
    Acc_NED = dcm.T@Acc # - np.array([0,0,9.81])  #dcm^-1 = dcm.T as dcm is orthonormal
    vel_NED = (Acc_NED[:-1] - acc_stored)*dt + vel_stored
    xy_NED = (vel_NED - vel_stored)*dt + xy_stored
    acc_stored = Acc_NED[:-1]
    vel_stored = vel_NED
    xy_stored = xy_NED
    return xy_NED

def zero_all():
    global acc_stored, vel_stored, xy_stored, pos_controller_error, previous_state_error
    acc_stored = np.zeros(2)
    vel_stored = np.zeros(2) 
    xy_stored = np.zeros(2)
    pos_controller_error = np.zeros(2)
    previous_state_error = np.zeros(4)


pos_controller_error = np.zeros(2) # x, y coordinates errors
# Returns desired roll and pitch 
def position_contoller(xy):
    # 1. World to body conversion
    # 2. Desired Pitch and roll
    global pos_controller_error
    xy_err = -xy                        #as we set desired xy to be [0,0]
    roll = helpers.get_pid(xy_err[0], pos_controller_error[0], config.pos_roll_P, config.pos_roll_I, config.pos_roll_D, dt)
    pitch = helpers.get_pid(xy_err[1], pos_controller_error[1], config.pos_pitch_P, config.pos_pitch_I, config.pos_pitch_D, dt)
    pos_controller_error = xy_err
    return roll, pitch



def get_throttle_rc(throttle_pid:float):
    print("Throttle PID:", throttle_pid)
    return np.clip(throttle_pid, 1000, 2000)

def get_yaw_rc(yaw_pid:float):
    print("Yaw PID:", yaw_pid)
    return np.clip(yaw_pid, 1000, 2000)


def get_pitch_rc(pitch_pid:float):
    print("Pitch PID:", pitch_pid)
    return np.clip(pitch_pid, 1000, 2000)


def get_roll_rc(roll_pid:float):
    print("Roll PID:", roll_pid)
    return np.clip(roll_pid, 1000, 2000)
