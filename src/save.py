import os, sys, glob, re, csv

import src.state as state

import src.mavlink as mavlink
import src.config as config


print("FILE save.py = ", __file__)
BASE_DIR = os.path.join(os.path.dirname(__file__), '../data')

# maybe better? - dir = base/session_%05d_f%05d_p
# now: dir = base/session_%08d_p%d   - session_idx + pid
def make_new_dir():
    dir_re = r'session_(\d+)_'
    base_dir_name = BASE_DIR.rstrip('/')

    dirs = glob.glob(os.path.join(base_dir_name, "session_*"))
    existing_idxs = []
    for dn in dirs:
        bn = os.path.basename(dn)
        m = re.match(dir_re, bn)
        if m:
            idx = int(m.group(1))
            existing_idxs += idx,

    print('existing_idxs=', existing_idxs)
    idx = max(existing_idxs, default=0) + 1

    pid = os.getpid()

    final_dir_name = os.path.join(base_dir_name, "session_%08d_p%d" % (idx, pid))

    if os.path.exists(final_dir_name):
        shutil.rmtree(final_dir_name)

    os.makedirs(final_dir_name, exist_ok=True)

    return final_dir_name


csv_file = None
def save_data(SaveQueue=None):
    global csv_file
    print('==== in save_data()')
    if not SaveQueue:
        return

 
    file_name = "log.csv"
    
    curr_dn = make_new_dir()
    print('=================MKDIR')
    
    # Open the file once
    file_path = os.path.join(curr_dn, file_name)
    
    csv_file = open(file_path, mode="w", newline="", buffering=1)  # Use buffering=1 for line buffering
    csv_writer = csv.writer(csv_file)

    #write a header row
    if config.HOVER:
        csv_writer.writerow(["actual_roll", "actual_pitch", "actual_yaw", "actual_alt", "desired_roll", "desired_pitch", "desired_yaw", "desired_alt", "ax", "ay", "az",
                         "filtered_ax", "filtered_ay", "filtered_az", "roll_rc", "pitch_rc", "yaw_rc", "throttle_rc", "time"])
    elif config.THROTTLE_TUNE:
        csv_writer.writerow(["current_alt", "desired_alt", "error", "throttle_rc", "throttle_pid", "time"])

    i = 0
    while True:
        item = SaveQueue.get()
        print(item)
        SaveQueue.task_done()
        csv_writer.writerow(item)
        if i==100:
            csv_file.flush()
            i=0
        i+=1

