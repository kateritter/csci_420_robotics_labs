#!/usr/bin/env python
import math, sys, copy, random
from velocity_pid import PID
import matplotlib.pyplot as plt
import numpy as np
def parse_paramstring(paramstring):
    try:
        return eval(paramstring)
    except:
        print("Error parsing params; returning default [0.0, 0.0, 0.0]")
        return [0.0, 0.0, 0.0]

def init_PID(paramstring):
    params = parse_paramstring(paramstring)
    print("Initialized PID with params p={} i={} d={}".format(params[0], params[1], params[2]))
    return PID(params[0], params[1], params[2])

def add_noise(x):
    return x + random.gauss(0.0, 0.3)

def distance(p1, p2):
    return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))

def draw_timestep(poses, waypoints, i):
    poses = copy.deepcopy(poses)
    # if len(poses) > 50:
    #     poses = poses[-50:]
    plt.ion()
    plt.clf()
    plt.plot([i[0] for i in poses], [i[1] for i in poses], 'bo--', linewidth=1, markersize=2)
    plt.plot([waypoint[0] for waypoint in waypoints[:i+1]], [waypoint[1] for waypoint in waypoints[:i+1]], 'ro')
    plt.xlim(-10,10)
    plt.ylim(-10,10)
    plt.show()
    plt.pause(0.1)

# To invoke for X: python debug_pid.py [px, ix, dx]
# To invoke for X and Y: python debug_pid.py [[px, ix, dx], [py, iy, dy]]
# takes in a set of waypoints and PID parameters and simulates a PID
def main():
    waypoints = np.array([[5,0],[5,5],[-5,5],[-5,-5],[5,-5]])
    epsilon = 0.01
    pose = [0.0, 0.0]
    dt = 0.1
    data = np.array(eval(sys.argv[1]))
    if len(data.shape) == 1:
        x_pid = PID(data[0], data[1], data[2])
        y_pid = None
    elif len(data.shape) == 2:
        x_pid = PID(data[0,0], data[0,1], data[0,2])
        y_pid = PID(data[1,0], data[1,1], data[1,2])
        epsilon *= 8
    else:
        raise ValueError("Data must be either a 1d array or a 2d array")
    poses = [copy.deepcopy(pose)]
    for i in range(len(waypoints)):
        cur_time_step = 0
        print("New waypoint:{}".format(waypoints[i]))
        while distance(pose, waypoints[i]) > epsilon:
            cur_time_step += 1
            print(f"Time on current waypoint: {cur_time_step}, dist: {distance(pose, waypoints[i]):0.3f}", end="\r")
            x_err = waypoints[i][0] - pose[0]
            pose[0] = add_noise(pose[0] + dt*x_pid.pid_loop(x_err, dt))
            if y_pid is not None:
                y_err = waypoints[i][1] - pose[1]
                pose[1] = add_noise(pose[1] + dt*y_pid.pid_loop(y_err, dt)) - 1
            poses.append(copy.deepcopy(pose))

            draw_timestep(poses, waypoints, i)
        print()
        print(f'Time steps to reach waypoint: {cur_time_step}')

    print("Finished waypoints!")
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()