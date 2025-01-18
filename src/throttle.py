import numpy as np
import queue
import time
#from scipy.integrate import trapezoid

import src.config as config
#import src.mavlink as mavlink
import src.type as types
import src.state as state
import src.helpers as helpers

keys = ["ID","current_alt", "desired_alt", "error", "throttle_rc", "throttle_pid", "time"]

dt = 0.01          #100Hz loop
t_last = None

desired_state = types.DesiredState()  #ALL NONE

previous_state_error = 0

vehicle_data = None

def throttle(CurrentAttitudeQueue=None, MavlinkSendQueue=None, SaveQueue=None):
    global desired_state, t_last, previous_state_error, dt, keys
    rc_channels = types.RC_CHANNELS()

    if not CurrentAttitudeQueue:
        return
    while True:
        try:
            vehicle_data = CurrentAttitudeQueue.get_nowait()
            current_state = vehicle_data['attitude']
            del vehicle_data["IMU"]
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


        current_state_vec = current_state.alt
        desired_state_vec = desired_state.alt

        current_state_error = desired_state_vec - current_state_vec

        throttle_pid = helpers.get_pid(current_state_error, previous_state_error, config.throttle_P, config.throttle_I, config.throttle_D, dt)

        previous_state_error = current_state_error


        throttle_rc = get_throttle_rc(throttle_pid)
        print("=========================================================")
        #pack controll channels into dictionary and update 
        controll_channels = controll_channels_pack(roll=0, pitch=0, yaw=0, throttle=throttle_rc)
        print(controll_channels)
        rc_channels.update_controll_channels(controll_channels)

        MavlinkSendQueue.put(rc_channels)
       
        log_data = ["CTRL"] + [current_state_vec] + [desired_state_vec] + [previous_state_error] + [throttle_rc] + [throttle_pid] + [time.time()]
        log = dict(zip(keys, log_data))
        try:
            if SaveQueue:
                SaveQueue.put(log)
        except queue.Full as err:
                        print("Dropped Controls Message", err)
            
        #create new instace for rc_channels
        rc_channels = types.RC_CHANNELS()  #is deleted in mavlink send thread
        #delete current state as they are the pointers on the objects we put in the queue in mavlink
        del current_state


def controll_channels_pack(roll, pitch, yaw, throttle):
    return {'roll': roll, 'pitch': pitch, 'yaw': yaw, 'throttle': throttle}



def zero_all():
    global previous_state_error
    previous_state_error = 0



def get_throttle_rc(throttle_pid:float):
    rc = helpers.map_pid_to_pwm_dynamic(throttle_pid,config.throttle_range)
    print("Throttle RC:", rc)
    return rc