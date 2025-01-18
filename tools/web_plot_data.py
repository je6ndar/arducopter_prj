from flask import Flask, render_template_string
from markupsafe import Markup
import os, sys
import pandas as pd
import plotly.graph_objects as go
import numpy as np

app = Flask(__name__)

# Helper functions
def get_latest_modified(path):
    entries = [os.path.join(path, entry) for entry in os.listdir(path)]
    entries = [entry for entry in entries if os.path.exists(entry)]
    if not entries:
        return None
    latest_entry = max(entries, key=os.path.getmtime)
    return latest_entry

def calculate_displacement(df):
    lat = df["lat"] / 10**7 * np.pi / 180
    lon = df["lon"] / 10**7 * np.pi / 180
    alt = df["alt"] / 10**3
    Re = 6378137
    e = 0.08181919
    N = Re / np.sqrt(1 - e**2 * np.sin(lat)**2)
    x = (N + alt) * np.cos(lat) * np.cos(lon)
    y = (N + alt) * np.cos(lat) * np.sin(lon)
    xy = np.column_stack((x, y))
    xy0 = xy[0]
    xy_centered = xy - xy0
    return xy_centered

@app.route("/")
def plot_data():
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

    pos = calculate_displacement(df_pos)
    time = np.array(df_pos["time"])

    # Plot Trajectory
    trajectory_fig = go.Figure()
    trajectory_fig.add_trace(
        go.Scatter(
            x=pos[:, 0],
            y=pos[:, 1],
            mode="markers",
            marker=dict(size=8, color=time, colorscale="Viridis", showscale=True),
            name="Trajectory"
        )
    )
    trajectory_fig.update_layout(title="Position (Trajectory)", xaxis_title="X", yaxis_title="Y")

    # Plot Roll
    roll_fig = go.Figure()
    roll_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["actual_roll"], mode="lines", name="Actual Roll"))
    roll_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["desired_roll"], mode="lines", name="Desired Roll"))
    roll_fig.update_layout(title="Roll", xaxis_title="Index", yaxis_title="Roll Value")

    # Plot Pitch
    pitch_fig = go.Figure()
    pitch_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["actual_pitch"], mode="lines", name="Actual Pitch"))
    pitch_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["desired_pitch"], mode="lines", name="Desired Pitch"))
    pitch_fig.update_layout(title="Pitch", xaxis_title="Index", yaxis_title="Pitch Value")

    # Plot Yaw
    yaw_fig = go.Figure()
    yaw_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["actual_yaw"], mode="lines", name="Actual Yaw"))
    yaw_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["desired_yaw"], mode="lines", name="Desired Yaw"))
    yaw_fig.update_layout(title="Yaw", xaxis_title="Index", yaxis_title="Yaw Value")

    #Plot Altitude
    alt_fig = go.Figure()
    alt_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["actual_alt"], mode="lines", name="Actual Altitude"))
    alt_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["desired_alt"], mode="lines", name="Desired Altitude"))
    alt_fig.update_layout(title="Altitude", xaxis_title="Index", yaxis_title="Altitude Value")

    # AccX, AccY, AccZ plots
    acc_fig = go.Figure()
    acc_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["ax"], mode="lines", name="ax"))
    acc_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["filtered_ax"], mode="lines", name="filtered_ax"))
    acc_fig.update_layout(title="AccX", xaxis_title="Index", yaxis_title="Acceleration Value")

    # RC plot
    rc_fig = go.Figure()
    rc_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["throttle_rc"], mode="lines", name="Throttle RC"))
    rc_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["pitch_rc"], mode="lines", name="Pitch RC"))
    rc_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["roll_rc"], mode="lines", name="Roll RC"))
    rc_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["yaw_rc"], mode="lines", name="Yaw RC"))
    rc_fig.update_layout(title="RC", xaxis_title="Index", yaxis_title="RC Value")

    # Roll PID plot
    roll_pid_fig = go.Figure()
    roll_pid_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["roll_pid"], mode="lines", name="Roll PID"))
    roll_pid_fig.update_layout(title="Roll PID", xaxis_title="Index", yaxis_title="PID Value")

    # Pitch PID plot
    pitch_pid_fig = go.Figure()
    pitch_pid_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["pitch_pid"], mode="lines", name="Pitch PID"))
    pitch_pid_fig.update_layout(title="Pitch PID", xaxis_title="Index", yaxis_title="PID Value")

    # Yaw PID plot
    yaw_pid_fig = go.Figure()
    yaw_pid_fig.add_trace(go.Scatter(x=df_ctrl.index, y=df_ctrl["yaw_pid"], mode="lines", name="Yaw PID"))
    yaw_pid_fig.update_layout(title="Yaw PID", xaxis_title="Index", yaxis_title="PID Value")


    # Generate HTML for each plot
    roll_html = Markup(roll_fig.to_html(full_html=False))
    pitch_html = Markup(pitch_fig.to_html(full_html=False))
    yaw_html = Markup(yaw_fig.to_html(full_html=False))
    alt_html = Markup(alt_fig.to_html(full_html=False))
    acc_html = Markup(acc_fig.to_html(full_html=False))
    rc_html = Markup(rc_fig.to_html(full_html=False))
    roll_pid_html = Markup(roll_pid_fig.to_html(full_html=False))
    pitch_pid_html = Markup(pitch_pid_fig.to_html(full_html=False))
    yaw_pid_html = Markup(yaw_pid_fig.to_html(full_html=False))
    trajectory_html = Markup(trajectory_fig.to_html(full_html=False))

    # Render the HTML template
    html_template = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Flight Data Plots</title>
        <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    </head>
    <body>
        <h1>Trajectory Plot</h1>
        <div>{{ trajectory_plot }}</div>
        <h1>Roll Plot</h1>
        <div>{{ roll_plot }}</div>
        <h1>Pitch Plot</h1>
        <div>{{ pitch_plot }}</div>
        <h1>Yaw Plot</h1>
        <div>{{ yaw_plot }}</div>
        <h1>Altitude Plot</h1>
        <div>{{ alt_plot }}</div>
        <h1>AccX</h1>
        <div>{{ acc_plot }}</div>
        <h1>RC</h1>
        <div>{{ rc_plot }}</div>
        <h1>Roll PID</h1>
        <div>{{ roll_pid_plot }}</div>
        <h1>Pitch PID</h1>
        <div>{{ pitch_pid_plot }}</div>
        <h1>Yaw PID</h1>
        <div>{{ yaw_pid_plot }}</div>
    </body>
    </html>
    """
    return render_template_string(
        html_template,
        trajectory_plot=trajectory_html,
        roll_plot=roll_html,
        pitch_plot=pitch_html,
        yaw_plot=yaw_html,
        alt_plot=alt_html,
        acc_plot=acc_html,
        rc_plot=rc_html,
        roll_pid_plot=roll_pid_html,
        pitch_pid_plot=pitch_pid_html,
        yaw_pid_plot=yaw_pid_html
    )

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
