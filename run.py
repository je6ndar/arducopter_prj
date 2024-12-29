
import state as state

import mavlink as mavlink
import config as config
import hover as hover

import queue
from threading import Thread

import time, sys, os

def run():
    MavlinkSendQueue = queue.Queue() # msgs to send via mavlink
    CurrentAttitudeQueue = queue.Queue()

    # recv/send data via mavlink
    MavlinkReceiveThread = Thread(target=mavlink.receive_mavlink, kwargs=dict(MavlinkSendQueue=MavlinkSendQueue))
    MavlinkSendThread = Thread(target=mavlink.send_mavlink, kwargs=dict(MavlinkSendQueue=MavlinkSendQueue))
    
    # controls threads
    HoverThread = Thread(target=hover.hover, kwargs=dict(CurrentAttitudeQueue=CurrentAttitudeQueue, MavlinkSendQueue=MavlinkSendQueue))
    
    threads = [MavlinkReceiveThread, MavlinkSendThread, HoverThread]
    
    for th in threads:
         th.start()

    # for debug
    #state.next_state(state.EV_RC_MED)
    #time.sleep(5)
    #state.next_state(state.EV_RC_HIGH)


if __name__ == "__main__":
    print("PID = ", os.getpid())
    run()
    #time.sleep(100)