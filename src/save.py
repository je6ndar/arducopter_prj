import os, sys, glob, re, csv, json
import shutil

import src.state as state
import src.helpers as helpers
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

file = None
def save_data(SaveQueue=None):
    global file
    print('==== in save_data()')
    if not SaveQueue:
        return

 
    file_name = "log.json"
    
    curr_dn = make_new_dir()
    print('=================MKDIR')
    
    # Open the file once
    file_path = os.path.join(curr_dn, file_name)

    file = open(file_path, 'a')

    i = 0
    while True:
        item = SaveQueue.get()
        json.dump(item, file, cls=helpers.CustomJSONEncoder)
        file.write('\n')
        if i==100:
            file.flush()
            i=0
        i+=1

