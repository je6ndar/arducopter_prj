
import src.state as state
import src.mavlink as mavlink
import src.config as config
import src.hover as hover
import src.save as save
import src.throttle as throttle

import queue
from threading import Thread

import time, sys, os

def run():
    MavlinkSendQueue = queue.Queue() # msgs to send via mavlink
    CurrentAttitudeQueue = queue.Queue()
    SaveQueue = queue.Queue()
    
    # recv/send data via mavlink
    MavlinkReceiveThread = Thread(target=mavlink.receive_mavlink, kwargs=dict(CurrentAttitudeQueue=CurrentAttitudeQueue, MavlinkSendQueue=MavlinkSendQueue))
    #MavlinkSendThread = Thread(target=mavlink.send_mavlink, kwargs=dict(MavlinkSendQueue=MavlinkSendQueue))
    
    # controls threads
    HoverThread = Thread(target=hover.hover, kwargs=dict(CurrentAttitudeQueue=CurrentAttitudeQueue, MavlinkSendQueue=MavlinkSendQueue, SaveQueue=SaveQueue))
    
    #Throttle Tune 
    ThrottleThread = Thread(target=throttle.throttle, kwargs=dict(CurrentAttitudeQueue=CurrentAttitudeQueue, MavlinkSendQueue=MavlinkSendQueue, SaveQueue=SaveQueue))

    #Save Thread
    SaveThread = Thread(target=save.save_data, kwargs=dict(SaveQueue=SaveQueue))

    threads = [MavlinkReceiveThread, SaveThread]
    
    if config.HOVER == True:
        threads.append(HoverThread)
    elif config.THROTTLE_TUNE == True:
        threads.append(ThrottleThread)
    for th in threads:
         th.start()

if __name__ == "__main__":
    try:
        print("PID = ", os.getpid())
        run()
    except KeyboardInterrupt:
        print("Interrupted by user")
        save.csv_file.close()
    #time.sleep(100)