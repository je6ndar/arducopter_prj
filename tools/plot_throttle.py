import os, sys
import pandas as pd
import matplotlib.pyplot as plt 

def get_latest_modified(path):
    entries = [os.path.join(path, entry) for entry in os.listdir(path)]
    
    entries = [entry for entry in entries if os.path.exists(entry)]
    
    if not entries:
        return None
    
    latest_entry = max(entries, key=os.path.getmtime)
    
    return latest_entry


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
        data_file = os.path.join(latest, 'log.csv')

    df = pd.read_csv(data_file)
    df = df.iloc[:-1]
    t0 = df["time"][0]
    dt = []
    for t in df["time"]:
        dt.append(t-t0)
        t0 = t
    dt_avg = sum(dt)/(len(dt))
    print("dt_avg:", dt_avg)
    fig, axes = plt.subplots(2,2, figsize=(10,8))
    
    axes[0, 0].plot(df.index, df['current_alt'], label='current_alt', color='b')
    axes[0, 0].plot(df.index, df['desired_alt'], label='desired_alt', color='r')
    axes[0, 0].set_title('Altitude')
    axes[0, 0].legend()

    axes[0, 1].plot(df.index, df['throttle_rc'], label='RC', color='b')
    axes[0, 1].set_title('Throttle RC')
    axes[0, 1].legend()

    axes[1, 0].plot(df.index, df['throttle_pid'], label='PID', color='b')
    axes[1, 0].set_title('Throttle PID')
    axes[1, 0].legend()

    axes[1, 1].plot(df.index, df['error'], label='Error', color='b')
    axes[1, 1].set_title('Error')
    axes[1, 1].legend()        

    plt.show()