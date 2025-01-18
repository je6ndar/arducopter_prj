import os, sys
import pandas as pd
import matplotlib.pyplot as plt
import json
import numpy as np

def get_latest_modified(path):
    entries = [os.path.join(path, entry) for entry in os.listdir(path)]
    
    entries = [entry for entry in entries if os.path.exists(entry)]
    
    if not entries:
        return None
    
    latest_entry = max(entries, key=os.path.getmtime)
    
    return latest_entry

def calculate_displacement(df):
    t_prev = df["time"][0]
    lat = df["lat"]/10**7 * np.pi/180  #latiude in rad
    lon = df["lon"]/10**7 * np.pi/180  #longitude in rad
    alt = df["alt"]/10**3  #alt in meters
    Re = 6378137    #radius of the Earth
    e = 0.08181919  #eccentricity of the Earth's ellipsoid 
    #prime vertical radius of curvature
    N = Re/np.sqrt(1-e**2*np.sin(lat)**2)
    x = (N+alt)*np.cos(lat)*np.cos(lon)
    y = (N+alt)*np.cos(lat)*np.sin(lon)
    xy = np.column_stack((x, y))
    xy0 = xy[0]
    xy_centered = xy-xy0
    return xy_centered

if __name__ == "__main__":
    
    if len(sys.argv)==2:
        data_file = sys.argv[1]
        if not os.path.exists(data_file):
            print("file doesn't exist")
            sys.exit()
    else:
        data_directory = os.path.join(os.path.dirname(__file__), '../data')
        latest = get_latest_modified(data_directory)
        if not latest:
            print("The data directory is empty.")
            sys.exit()
        data_file = os.path.join(latest, 'log.json')
            
    df = pd.read_json(data_file, lines=True)
    df = df.iloc[:-1]
    df_ctrl = df[df["ID"] == "CTRL"]
    df_pos = df[df["ID"] == "POS"]
    t0 = df["time"][0]

    # for t in df["time"]:
    #     dt.append(t-t0)
    #     t0 = t
    # dt_avg = sum(dt)/(len(dt))
    # print("dt_avg:", dt_avg)
    pos = calculate_displacement(df_pos)
    
    fig, axes = plt.subplots(2,2, figsize=(10,8))

    axes[0, 0].plot(df_ctrl.index, df_ctrl['actual_roll'], label='actual_roll', color='b')
    axes[0, 0].plot(df_ctrl.index, df_ctrl['desired_roll'], label='desired_roll', color='r')
    axes[0, 0].set_title('Roll')
    axes[0, 0].legend()

    axes[0, 1].plot(df_ctrl.index, df_ctrl['actual_pitch'], label='actual_pitch', color='b')
    axes[0, 1].plot(df_ctrl.index, df_ctrl['desired_pitch'], label='desired_pitch', color='r')
    axes[0, 1].set_title('Pitch')
    axes[0, 1].legend()

    axes[1, 0].plot(df_ctrl.index, df_ctrl['actual_yaw'], label='actual_yaw', color='b')
    axes[1, 0].plot(df_ctrl.index, df_ctrl['desired_yaw'], label='desired_yaw', color='r')
    axes[1, 0].set_title('Yaw')
    axes[1, 0].legend()

    axes[1, 1].plot(df_ctrl.index, df_ctrl['actual_alt'], label='actual_alt', color='b')
    axes[1, 1].plot(df_ctrl.index, df_ctrl['desired_alt'], label='desired_alt', color='r')
    axes[1, 1].set_title('Altitude')
    axes[1, 1].legend()

    fig1, axes1 = plt.subplots(2,2, figsize=(10,8))
    axes1[0, 0].plot(df_ctrl.index, df_ctrl['ax'], label='ax', color='b')
    axes1[0, 0].plot(df_ctrl.index, df_ctrl['filtered_ax'], label='filtered_ax', color='r')
    axes1[0, 0].set_title('AccX')
    axes1[0, 0].legend()

    axes1[0, 1].plot(df_ctrl.index, df_ctrl['ax'], label='ax', color='b')
    axes1[0, 1].plot(df_ctrl.index, df_ctrl['filtered_ax'], label='filtered_ax', color='r')
    axes1[0, 1].set_title('AccY')
    axes1[0, 1].legend()

    axes1[1, 0].plot(df_ctrl.index, df_ctrl['ax'], label='ax', color='b')
    axes1[1, 0].plot(df_ctrl.index, df_ctrl['filtered_ax'], label='filtered_ax', color='r')
    axes1[1, 0].set_title('AccZ')
    axes1[1, 0].legend()

    axes1[1, 1].plot(df_ctrl.index, df_ctrl['throttle_rc'], label='throttle_rc', color='b')
    axes1[1, 1].plot(df_ctrl.index, df_ctrl['pitch_rc'], label='pitch_rc', color='r')
    axes1[1, 1].plot(df_ctrl.index, df_ctrl['roll_rc'], label='roll_rc', color='g')
    axes1[1, 1].plot(df_ctrl.index, df_ctrl['yaw_rc'], label='yaw_rc', color='y')
    axes1[1, 1].set_title('RC')
    axes1[1, 1].legend()

    fig2, axes2 = plt.subplots(2,2, figsize=(10,8))
    axes2[0, 0].plot(df_ctrl.index, df_ctrl['roll_pid'], color='b')
    axes2[0, 0].set_title('Roll PID')

    axes2[0, 1].plot(df_ctrl.index, df_ctrl['pitch_pid'], color='b')
    axes2[0, 1].set_title('Pitch PID')

    axes2[1, 0].plot(df_ctrl.index, df_ctrl['yaw_pid'], color='b')
    axes2[1, 0].set_title('Yaw PID')

    axes2[1, 1].plot(df_ctrl.index, df_ctrl['throttle_pid'], color='b')
    axes2[1, 1].set_title('Throttle PID')
    
    time = np.array(df_pos["time"])  # Convert time to a NumPy array for coloring
    fig3, axes3 = plt.subplots(1,1, figsize=(5,4))
    axes3.grid(True)
    #axes3.plot(pos[:,0],pos[:,1], 'o')
    axes3.scatter(pos[:,0],pos[:,1], c=time, cmap='viridis', label='Trajectory')
    axes3.set_title('Position')
    
    plt.tight_layout()

    plt.show()