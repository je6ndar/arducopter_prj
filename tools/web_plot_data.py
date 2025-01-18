from flask import Flask, render_template_string
from markupsafe import Markup
import os
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
    # Load data
    data_directory = os.path.join(os.path.dirname(__file__), '../data')
    latest = get_latest_modified(data_directory)
    if not latest:
        return "The data directory is empty."

    data_file = os.path.join(latest, 'log.json')
    df = pd.read_json(data_file, lines=True)
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

    # Generate HTML for each plot
    trajectory_html = Markup(trajectory_fig.to_html(full_html=False))
    roll_html = Markup(roll_fig.to_html(full_html=False))
    pitch_html = Markup(pitch_fig.to_html(full_html=False))
    yaw_html = Markup(yaw_fig.to_html(full_html=False))

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
    </body>
    </html>
    """
    return render_template_string(
        html_template,
        trajectory_plot=trajectory_html,
        roll_plot=roll_html,
        pitch_plot=pitch_html,
        yaw_plot=yaw_html
    )

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
