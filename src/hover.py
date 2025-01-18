import numpy as np
import queue
import time
#from scipy.integrate import trapezoid

import src.config as config
#import src.mavlink as mavlink
import src.type as types
import src.state as state
import src.helpers as helpers

'''
Raw PID -> clamp Raw PID -> map it onto the pwm value 
'''

'''
Position estimate:

Euler angles -> rotational matrix -> rot matrix.T@acc -> double integration 
'''
keys = ["ID","actual_roll", "actual_pitch", "actual_yaw", "actual_alt", "desired_roll", "desired_pitch", "desired_yaw", "desired_alt", "ax", "ay", "az",
                         "filtered_ax", "filtered_ay", "filtered_az", "roll_rc", "pitch_rc", "yaw_rc", "throttle_rc", "roll_pid", "pitch_pid", "yaw_pid", "throttle_pid", "time"]


dt = 0.01          #100Hz loop
t_last = None
scale_imu = 0.00981

desired_state = types.DesiredState()  #ALL NONE
#BATCH_SIZE = 4
#accel_buffer = types.CircularBuffer(BATCH_SIZE)
previous_state_error = np.zeros(4)
acc_filtered_previous = np.zeros(3)

vehicle_data = None

def hover(CurrentAttitudeQueue=None, MavlinkSendQueue=None, SaveQueue=None):
    global desired_state, previous_state_error, acc_filtered_previous, dt, t_last, keys#, current_state, vehicle_data
    rc_channels = types.RC_CHANNELS()

    if not CurrentAttitudeQueue:
        return
    while True:
        try:
            vehicle_data = CurrentAttitudeQueue.get_nowait()
            current_state = vehicle_data['attitude']
            current_imu = vehicle_data['IMU']
            if t_last:
                dt = (vehicle_data['time']- t_last)
            t_last = vehicle_data['time']
            if not desired_state.alt:
                print("+++++++++++++++++++++++INIT+++++++++++++++++++++++++")
                desired_state.set_initial_condition(current_state)       # Update desired_state with current_state
            CurrentAttitudeQueue.task_done()
        except queue.Empty as error:
            if state.STATE==state.IDLE:
                desired_state.clear() #Sets all to None
                zero_all()  #function that zeros all stored data related to state estimation and errors
                continue
            else:
                continue
        current_imu.scale(scale_imu)
        Acc = helpers.low_pass_filter(current_imu.get_acc_vec(), acc_filtered_previous, 0.7)

        acc_filtered_previous = Acc
        
        ''' 
        accel_buffer.add(Acc)
        if accel_buffer.is_full():
            xy = get_coordinate_estimate(current_state, accel_buffer)
            roll, pitch = position_contoller(current_state, xy)
        '''
        xy = get_coordinate_estimate(current_state, Acc)
        desired_roll, desired_pitch = position_contoller(xy)
        desired_state.update_desired_roll_pitch(desired_roll, desired_pitch)   #we have complete desired state at this point 
        # print("Desired State", desired_state.roll, desired_state.pitch, desired_state.yaw, desired_state.alt)
        # print("Current State", current_state.roll, current_state.pitch, current_state.yaw, current_state.alt)
        # print("Frequency:", 1/dt)
        #Now get difference between desired and actual state and run pid loops
        current_state_vec = current_state.get_state_vec()
        desired_state_vec = desired_state.get_state_vec()

        current_state_error = desired_state_vec - current_state_vec   #roll, pitch, yaw, alt 

        roll_pid =helpers.get_pid(current_state_error[0], previous_state_error[0], config.roll_P, config.roll_I, config.roll_D, dt)
        yaw_pid = helpers.get_pid(current_state_error[1], previous_state_error[1], config.yaw_P, config.yaw_I, config.yaw_D, dt)
        pitch_pid = helpers.get_pid(current_state_error[2], previous_state_error[2], config.pitch_P, config.pitch_I, config.pitch_D, dt)
        throttle_pid = helpers.get_pid(current_state_error[3], previous_state_error[3], config.throttle_P, config.throttle_I, config.throttle_D, dt)

        previous_state_error = current_state_error

        pid_vec = [roll_pid, pitch_pid, yaw_pid, throttle_pid]

        roll_rc = get_roll_rc(roll_pid)
        pitch_rc = get_pitch_rc(pitch_pid)
        throttle_rc = get_throttle_rc(throttle_pid)
        yaw_rc = get_yaw_rc(yaw_pid)
        print("=========================================================")
        #pack controll channels into dictionary and update 
        controll_channels = controll_channels_pack(roll=roll_rc, pitch=pitch_rc, yaw=yaw_rc, throttle=throttle_rc)
        rc_channels.update_controll_channels(controll_channels)

        MavlinkSendQueue.put(rc_channels)
        #actual_roll actual_pitch actual_yaw actual_alt desired_roll desired_pitch desired_yaw desired_alt ax ay az filtered_ax filtered_ay filtered_az, roll_rc, pitch_rc, yaw_rc, throttle_rc, time?
        #log_data_vec = [current_state.roll, current_state.pitch, current_state.yaw, current_state.alt, desired_state.roll, desired_pitch.pitch, desired_state.yaw]
        
        log_data = ["CTRL"] + list(current_state_vec) + list(desired_state_vec) + list(current_imu.get_acc_vec()) + list(Acc) + list(rc_channels.get_rpyt_vec()) + pid_vec + [time.time()]
        log = dict(zip(keys,log_data))
        try:
            if SaveQueue:
                SaveQueue.put(log)
        except queue.Full as err:
                        print("Dropped Controls Message", err)
            
        #create new instace for rc_channels
        rc_channels = types.RC_CHANNELS()  #is deleted in mavlink send thread
        #delete current state as they are the pointers on the objects we put in the queue in mavlink
        del current_state
        del current_imu


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
    dcm = helpers.euler_to_dcm(rpy)   #input [roll, pitch, yaw] 
    dcm = np.array(dcm)
    #3d vector of acceleration in NED frame
    Acc_NED = dcm.T@Acc # - np.array([0,0,9.81])  #dcm^-1 = dcm.T as dcm is orthonormal
    vel_NED = (Acc_NED[:-1] - acc_stored)*dt + vel_stored
    xy_NED = (vel_NED - vel_stored)*dt + xy_stored
    acc_stored = Acc_NED[:-1]
    vel_stored = vel_NED
    xy_stored = xy_NED
    #print(xy_NED)
    return xy_NED

def zero_all():
    global acc_stored, vel_stored, xy_stored, pos_controller_error, previous_state_error, acc_filtered_previous
    acc_stored = np.zeros(2)
    vel_stored = np.zeros(2) 
    xy_stored = np.zeros(2)
    pos_controller_error = np.zeros(2)
    previous_state_error = np.zeros(4)
    acc_filtered_previous = np.zeros(3)



#TODO - Map pid roll/pitch output onto angle
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
    rc = helpers.map_pid_to_pwm_dynamic(throttle_pid,config.throttle_range)
    return rc

def get_yaw_rc(yaw_pid:float):
    rc = helpers.map_pid_to_pwm_dynamic(yaw_pid,config.yaw_range)
    return rc


def get_pitch_rc(pitch_pid:float):
    rc = helpers.map_pid_to_pwm_dynamic(pitch_pid,config.pitch_range)
    return rc

def get_roll_rc(roll_pid:float):
    rc = helpers.map_pid_to_pwm_dynamic(roll_pid,config.roll_range)
    return rc