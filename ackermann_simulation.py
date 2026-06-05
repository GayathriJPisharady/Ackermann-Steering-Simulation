import matplotlib
matplotlib.use('TkAgg')

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
import math

# -----------------------------
# PARAMETERS
# -----------------------------
wheelbase = 2.4
track_width = 1.4
speed = 1.0
running = True
frame_idx = 0.0

# data for graph
inner_hist = []
outer_hist = []

# -----------------------------
# ROAD
# -----------------------------
t = np.linspace(0, 60, 800)
road_x = t
road_y = 4*np.sin(t/6) + 2*np.sin(t/12)

dx = np.gradient(road_x)
dy = np.gradient(road_y)
angles = np.arctan2(dy, dx)

# -----------------------------
# FIGURE
# -----------------------------
fig, ax = plt.subplots(figsize=(10,6))
plt.subplots_adjust(bottom=0.3)

ax.set_title("Correct Ackermann Steering Simulation")

# road
lane = 2.5
norm = np.sqrt(dx**2 + dy**2)
nx = -dy / norm
ny = dx / norm

ax.plot(road_x + lane*nx, road_y + lane*ny, 'gray', linewidth=3)
ax.plot(road_x - lane*nx, road_y - lane*ny, 'gray', linewidth=3)
ax.plot(road_x, road_y, 'yellow', linestyle='--')

# car + wheels
car_body, = ax.plot([], [], 'blue', linewidth=3)
wheels = [ax.plot([], [], 'black', linewidth=3)[0] for _ in range(4)]

info = ax.text(0.02, 0.95, "", transform=ax.transAxes,
               bbox=dict(facecolor='white', alpha=0.7))

# -----------------------------
# TRANSFORM
# -----------------------------
def transform(x, y, a, lx, ly):
    X = x + lx*np.cos(a) - ly*np.sin(a)
    Y = y + lx*np.sin(a) + ly*np.cos(a)
    return X, Y

# -----------------------------
# CAR + WHEEL
# -----------------------------
def get_car(x,y,a):
    L,W = wheelbase, track_width
    pts = [(-L/2,-W/2),(L/2,-W/2),(L/2,W/2),
           (-L/2,W/2),(-L/2,-W/2)]
    return np.array([transform(x,y,a,px,py) for px,py in pts])

def get_wheel(x,y,a,s):
    pts = [(-0.4,0),(0.4,0)]
    return np.array([transform(x,y,a+s,px,py) for px,py in pts])

# -----------------------------
# ANIMATION
# -----------------------------
def update(frame):
    global frame_idx

    if running:
        frame_idx += speed
        if frame_idx >= len(road_x)-2:
            frame_idx = 0

    i = int(frame_idx)

    x,y = road_x[i], road_y[i]
    angle = angles[i]

    # curvature
    if i > 5:
        k = (angles[i] - angles[i-5]) / 5
    else:
        k = 0

    if abs(k) < 1e-4:
        R = 1e6
    else:
        R = 1/k

    # -----------------------------
    # TRUE ACKERMANN
    # -----------------------------
    if abs(R) > 1e5:
        left = right = 0
    else:
        theta_inner = math.atan(wheelbase/(R - track_width/2))
        theta_outer = math.atan(wheelbase/(R + track_width/2))

        if k > 0:  # left turn
            left = theta_inner
            right = theta_outer
        else:      # right turn
            left = theta_outer
            right = theta_inner

    # store data
    inner_hist.append(math.degrees(left))
    outer_hist.append(math.degrees(right))

    # draw car
    body = get_car(x,y,angle)
    car_body.set_data(body[:,0], body[:,1])

    # wheel local positions (correct layout)
    L,W = wheelbase, track_width
    wheel_local = [
        (-L/2, -W/2),  # rear left
        (-L/2,  W/2),  # rear right
        ( L/2, -W/2),  # front left
        ( L/2,  W/2)   # front right
    ]

    for j,(lx,ly) in enumerate(wheel_local):
        wx, wy = transform(x,y,angle,lx,ly)

        if j == 2:
            steer = left
        elif j == 3:
            steer = right
        else:
            steer = 0

        w = get_wheel(wx,wy,angle,steer)
        wheels[j].set_data(w[:,0], w[:,1])

    info.set_text(
        f"Inner: {math.degrees(left):.1f}°\n"
        f"Outer: {math.degrees(right):.1f}°\n"
        f"Speed: {speed:.1f}"
    )

    return [car_body] + wheels + [info]

ani = FuncAnimation(fig, update, interval=30)

# -----------------------------
# UI
# -----------------------------
ax_speed = plt.axes([0.2,0.2,0.6,0.03])
speed_slider = Slider(ax_speed, "Speed", 0.5, 5, valinit=speed)

def update_speed(val):
    global speed
    speed = speed_slider.val

speed_slider.on_changed(update_speed)

# buttons
ax_play = plt.axes([0.2,0.08,0.15,0.06])
ax_pause = plt.axes([0.4,0.08,0.15,0.06])
ax_graph = plt.axes([0.6,0.08,0.2,0.06])

btn_play = Button(ax_play,"Play")
btn_pause = Button(ax_pause,"Pause")
btn_graph = Button(ax_graph,"Show Graph")

def play(event):
    global running
    running = True

def pause(event):
    global running
    running = False

def show_graph(event):
    plt.figure()
    plt.plot(inner_hist,label="Inner Angle")
    plt.plot(outer_hist,label="Outer Angle")
    plt.legend()
    plt.title("Steering Angles vs Time")
    plt.xlabel("Time")
    plt.ylabel("Angle (deg)")
    plt.grid()
    plt.show()

btn_play.on_clicked(play)
btn_pause.on_clicked(pause)
btn_graph.on_clicked(show_graph)

ax.set_aspect('equal')
plt.show()