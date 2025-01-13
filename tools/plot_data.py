import os
import pandas as pd
import matplotlib.pyplot as plt 

def get_latest_modified(path):
    entries = [os.path.join(path, entry) for entry in os.listdir(path)]
    
    entries = [entry for entry in entries if os.path.exists(entry)]
    
    if not entries:
        return None
    
    latest_entry = max(entries, key=os.path.getmtime)
    
    return latest_entry

#data frame structure is defined in notes/log.mb
def parse_data_frame(df):
    actual_roll = df['actual_roll']
    data_dict = dict(actual_roll=actual_roll)
    return data_dict

if __name__ == "__main__":
    data_directory = os.path.join(os.path.dirname(__file__), '../data')
    latest = get_latest_modified(data_directory)
    if not latest:
        print("The data directory is empty.")
    else:
        data_file = os.path.join(latest, 'log.csv')
        df = pd.read_csv(data_file)
        t0 = df["time"][0]
        dt = []
        for t in df["time"]:
            dt.append(t-t0)
            t0 = t
        dt_avg = sum(dt)/(len(dt))
        print("dt_avg:", dt_avg)
        fig, axes = plt.subplots(2,2, figsize=(10,8))

        axes[0, 0].plot(df.index, df['actual_roll'], label='actual_roll', color='b')
        axes[0, 0].plot(df.index, df['desired_roll'], label='desired_roll', color='r')
        axes[0, 0].set_title('Roll')
        axes[0, 0].legend()

        axes[0, 1].plot(df.index, df['actual_pitch'], label='actual_pitch', color='b')
        axes[0, 1].plot(df.index, df['desired_pitch'], label='desired_pitch', color='r')
        axes[0, 1].set_title('Pitch')
        axes[0, 1].legend()

        axes[1, 0].plot(df.index, df['actual_yaw'], label='actual_yaw', color='b')
        axes[1, 0].plot(df.index, df['desired_yaw'], label='desired_yaw', color='r')
        axes[1, 0].set_title('Yaw')
        axes[1, 0].legend()

        axes[1, 1].plot(df.index, df['actual_alt'], label='actual_alt', color='b')
        axes[1, 1].plot(df.index, df['desired_alt'], label='desired_alt', color='r')
        axes[1, 1].set_title('Altitude')
        axes[1, 1].legend()

        fig1, axes1 = plt.subplots(2,2, figsize=(10,8))
        axes1[0, 0].plot(df.index, df['ax'], label='ax', color='b')
        axes1[0, 0].plot(df.index, df['filtered_ax'], label='filtered_ax', color='r')
        axes1[0, 0].set_title('AccX')
        axes1[0, 0].legend()

        axes1[0, 1].plot(df.index, df['ax'], label='ax', color='b')
        axes1[0, 1].plot(df.index, df['filtered_ax'], label='filtered_ax', color='r')
        axes1[0, 1].set_title('AccY')
        axes1[0, 1].legend()

        axes1[1, 0].plot(df.index, df['ax'], label='ax', color='b')
        axes1[1, 0].plot(df.index, df['filtered_ax'], label='filtered_ax', color='r')
        axes1[1, 0].set_title('AccZ')
        axes1[1, 0].legend()

        axes1[1, 1].plot(df.index, df['throttle_rc'], label='throttle_rc', color='b')
        axes1[1, 1].plot(df.index, df['pitch_rc'], label='pitch_rc', color='r')
        axes1[1, 1].plot(df.index, df['roll_rc'], label='roll_rc', color='g')
        axes1[1, 1].plot(df.index, df['yaw_rc'], label='yaw_rc', color='y')
        axes1[1, 1].set_title('Acc')
        axes1[1, 1].legend()
        plt.tight_layout()

        plt.show()