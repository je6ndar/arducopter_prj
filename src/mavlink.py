from pymavlink import mavutil
import time
import queue
from threading import Thread
import json, os

import config as config
import state as state
import helpers as helpers
import type as types


SYSTEM_ID = None
SYSTEM_COMPONENT = None
COMPONENT_ID = mavutil.mavlink.MAV_COMP_ID_USER10

MAVCONN = None
LAST_RECV_MSG_TIME = None

# when connecting, it will iterate over the following rates
#MAVLINK_BAUD_RATES = [57600, 115200, 921600]
MAVLINK_BAUD_RATES = [921600, 115200, 57600]
MAVLINK_BAUD_RATES_worked = [] # used to save worked baud and quikly restarts if needed

MAVLINK_SERIAL = '/dev/ttyAMA0'


MAVLINK_SAVE_FN = 'mavlink.jsons'

ATT_FREQ_Hz = 100

MAX_TIME_WITHOUT_MSG = 2.0 # in seconds

MODE_MAP = [
    'STABILIZE', 'ACRO', 'ALT HOLD', 'AUTO', 'GUIDED', 'LOITER', 'RTL', 'CIRCLE', 'LAND', 'DRIFT', 'SPORT',
    'FLIP', 'AUTOTUNE', 'POSHOLD', 'BRAKE', 'THROW', 'AVOID ADSB', 'GUIDED NOGPS', 'SMARTRTL', 'FLOWHOLD',
    'FOLLOW', 'ZIGZAG', 'SYSTEMID', 'AUTOROTATE', 'AUTO RTL', 'TURTLE'
]


MSG_TYPES = [
    'ATTITUDE',
    'VFR_HUD',
    'EKF_STATUS_REPORT',
    'RAW_IMU',
    'TIMESYNC',
    'HEARTBEAT',
    #'MAVLINK_MSG_ID_RC_CHANNELS_SCALED', 'MAVLINK_MSG_ID_RC_CHANNELS_RAW'
    'SERVO_OUTPUT_RAW',
    #'RC_CHANNELS'
]



# read from MAVCONN -> change STATE, put into SaveQueue
# read from MavlinkSendQueue, send to MAVCONN
"""

comm with mavlink
  conn to mavlink (read GPS, ATT, etc.)
  queue to write messages (dummy GPS_INPUT, or VIS msgs)
 loop:
   read blocking from mavlink conn
    if GPS etc -> SaveMavlikQueue
   get_nowait from queue
    if any -> send to conn
"""
attitude = types.CurrentState()
imu = types.IMU()

def receive_mavlink(CurrentAttitudeQueue, SaveQueue=None):
    global MAVCONN, LAST_RECV_MSG_TIME, FLIGHT_MODE, PREV_FLIGHT_MODE, attitude
    
    
    while True:
        if not MAVCONN:
            try:
                t_conn1 = time.time()
                MAVCONN = init_mavlink()
                t_conn2 = time.time()
                print("DT mavlink-conn-init: %.3f" % (t_conn2-t_conn1))
            except RuntimeError as err:
                print("ERROR while init MAVLINK:", err)
                print("Try reinit in 1 sec")
                MAVCONN = None
                LAST_RECV_MSG_TIME = None


        if MAVCONN:
            in_msg = MAVCONN.recv_match(type=MSG_TYPES, blocking=True)
        else:
            in_msg = None

        if in_msg is not None:
            LAST_RECV_MSG_TIME = time.time()
            current_attitude = {'roll': None, 'pitch': None, 'yaw': None, 'alt': None}  #Set current attitude to None every new cycle of mavlink message acquisition    

            # Get current flight mode 
            if in_msg.get_type() == 'HEARTBEAT':
                current_mode = get_flight_mode(in_msg)
                print("CurrentFlightMode:", current_mode)
                if current_mode != FLIGHT_MODE:
                    FLIGHT_MODE = current_mode

            if in_msg.get_type() == 'RAW_IMU':
                acc = [in_msg.xacc, in_msg.yacc, in_msg.zacc]
                imu.update(acc)

            if in_msg.get_type() == 'ATTITUDE':
                current_attitude['roll'] = in_msg.roll                
                current_attitude['pitch'] = in_msg.pitch
                current_attitude['yaw'] = in_msg.yaw
            
            if in_msg.get_type() == 'VFR_HUD':
                current_attitude['alt'] = in_msg.alt

            ## State change based on SERVO_OUTPUT_RAW
            if in_msg.get_type() == 'SERVO_OUTPUT_RAW':
                rc_event = helpers.servo_raw_to_rc_level(in_msg)
                if rc_event == state.EV_RC_HIGH:                                                             #REDO
                    if FLIGHT_MODE != 'GUIDED':
                        PREV_FLIGHT_MODE = FLIGHT_MODE
                        set_mode(MAVCONN, 'GUIDED')
                elif rc_event in [state.EV_RC_LOW, state.EV_RC_MED]:
                    if FLIGHT_MODE == 'GUIDED' and PREV_FLIGHT_MODE:
                        set_mode(MAVCONN, PREV_FLIGHT_MODE)
                        PREV_FLIGHT_MODE = None


                if rc_event in [state.EV_RC_LOW, state.EV_RC_MED, state.EV_RC_HIGH]:                         #MED?
                    state.next_state(rc_event)
                
                if state.STATE==state.HOVER:
                    attitude.update(current_attitude)                           # updates values of attitude, keeps previous if new is missing
                    vehicle_data = {'attitude':attitude,'IMU':imu} 
                    CurrentAttitudeQueue.put(vehicle_data)
            try:
                if SaveQueue:
                    SaveQueue.put_nowait(MavlinkMessageItem(in_msg))
            except queue.Full as err:

                print("Dropped Mavlink Message", err)
        else:
            if LAST_RECV_MSG_TIME is None or time.time() - LAST_RECV_MSG_TIME > MAX_TIME_WITHOUT_MSG:

                MAVCONN = None
                LAST_RECV_MSG_TIME = None


def send_mavlink(MavlinkSendQueue=None):
    if MavlinkSendQueue is not None:
        try:
            out_msg = MavlinkSendQueue.get_nowait()
            out_msg_mavlink = convert_msg(out_msg)
            if MAVCONN and out_msg_mavlink:
                res = MAVCONN.mav.send(out_msg_mavlink)
            MavlinkSendQueue.task_done()
        except queue.Empty as err:
            pass

def set_mode(master, mode):
    if mode not in master.mode_mapping():
        print(f"Unknown mode: {mode}")
        return

    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

def get_flight_mode(msg):
    mode_num = msg.custom_mode
    mode = MODE_MAP[mode_num]
    return mode


# wrapper class to save into file + store os_time
class MavlinkMessageItem:
    def __init__(self, msg):
        self.msg = msg
        self.os_time = time.time()

    def save(self, out_dn):
        fn = os.path.join(out_dn, MAVLINK_SAVE_FN)
        with open(fn, 'at') as out_f:
            d = self.msg.to_dict()
            d['os_time'] = self.os_time
            json.dump(d, out_f)
            out_f.write('\n')
            #out_f.flush()


def convert_msg(item):
    msg = None
    # https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
    if type(item) == types.RC_CHANNELS:
        rc = item.RC
        msg = create_rc_msg(rc)
        pass
    # https://mavlink.io/en/messages/common.html#STATUSTEXT
    elif type(item) == types.Status:
        msg = MAVCONN.mav.statustext_encode(
            severity=mavutil.mavlink.MAV_SEVERITY_INFO,
            text = item.text.encode(encoding='ascii', errors='replace')
        )

    return msg


# TODO - time?
def create_rc_msg(rc):
    if not MAVCONN:
        return None
    
    rc1,rc2,rc3,rc4,rc5,rc6,rc7,rc8,rc9,rc10,rc11,rc12,rc13,rc14,rc15,rc16,rc17,rc18 = rc 
    
    msg = msg = MAVCONN.mav.rc_channels_override_send(
        #target_system = MAVCONN.target_system, 
        #target_component = MAVCONN.target_component,
        target_system = 0,
        target_component = 0, # mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        chan1_raw = rc1,
        chan2_raw = rc2,
        chan3_raw = rc3,
        chan4_raw = rc4,
        chan5_raw = rc5,
        chan6_raw = rc6,
        chan7_raw = rc7,
        chan8_raw = rc8,
        chan9_raw = rc9,
        chan10_raw = rc10,
        chan11_raw = rc11,
        chan12_raw = rc12,
        chan13_raw = rc13,
        chan14_raw = rc14,
        chan15_raw = rc15,
        chan16_raw = rc16,
        chan17_raw = rc17,
        chan18_raw = rc18
    )

    return msg



def init_mavlink():
    heartbeat_msg = None
    global MAVLINK_BAUD_RATES_worked


    if config.SITL == True:
        print("MAVLINK CONNECTING TO SITL ")
        for i in range(5):
            mavconn = mavutil.mavlink_connection('udp::14551')
            heartbeat_msg = mavconn.wait_heartbeat(timeout=2) 
            if heartbeat_msg is not None:
                print("FOUND SITL")
                break

    else:
        for baud in MAVLINK_BAUD_RATES_worked + MAVLINK_BAUD_RATES:
            print("MAVLINK try baud=", baud)
            mavconn = mavutil.mavlink_connection(MAVLINK_SERIAL, baud=baud)
            heartbeat_msg = mavconn.wait_heartbeat(timeout=2) 
            if heartbeat_msg is not None:
                MAVLINK_BAUD_RATES_worked = [baud]
                break

        # not connected
    if not heartbeat_msg:
        return None

    #mavconn = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    #mavconn.wait_heartbeat() 

    #mavconn = mavutil.mavlink_connection('/dev/serial0', baud=57600)
    global SYSTEM_ID, SYSTEM_COMPONENT
    SYSTEM_ID = mavconn.target_system
    SYSTEM_COMPONENT = mavconn.target_component

    print("Heartbeat from system (system %u component %u)" % (mavconn.target_system, mavconn.target_component))
    #perform_timesync_from_systime(mavconn)

    request_message_interval(mavconn, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, ATT_FREQ_Hz)
    request_message_interval(mavconn, mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, ATT_FREQ_Hz)
    request_message_interval(mavconn, mavutil.mavlink.MAVLINK_MSG_RAW_IMU, ATT_FREQ_Hz)

    return mavconn


def request_message_interval(conn, message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )
