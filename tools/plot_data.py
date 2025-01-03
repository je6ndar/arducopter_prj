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

        plt.tight_layout()

        plt.show()