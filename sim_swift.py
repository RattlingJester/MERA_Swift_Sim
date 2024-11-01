import json
import threading
from os import path
from math import degrees

import numpy as np
import roboticstoolbox as rtb
import roboticstoolbox.tools.trajectory as rtb_tools_trajectory
import spatialgeometry as sg
import swift
from numpy.typing import ArrayLike
from spatialmath import SE3

import MERA5002
import BORUNTE2110

np.set_printoptions(suppress=True)

# robot = MERA5002.X_5002()
robot = BORUNTE2110.BRTIRUS2110A()

# gripper = robot.grippers[0]
env = swift.Swift()

JsonPoints = {
    "action": 10, # Action type 4: Free Path, 10: Posture Linear, 17: Posture Curve
    "ckStatus": "63", # Axis movement mask, 0X3F == 63 represents 6 axes; 0xFF == 255 represents 8 axes
    "delay": "0.000", # delay before action
    "insertedIndex": int,
    "points": [
        {
            "pointName": "",
            "pos": {
                "m0": int,
                "m1": int,
                "m2": int,
                "m3": int,
                "m4": int,
                "m5": int
            }
        }
    ],
    "smooth": 0, # Smoothness level (IDK what it is)
    "speed": "100", # Persent??
    "toolCoord": 262146 # dunno what it is
}
f = open("./test.json", "w").close()

dt = 0.02 # Time step
dtraj = 10 # Trajectory visualisation step
STacc = 0.5 # Acceleration
qdmax = np.zeros(shape=[6,1], dtype=float)
qdmax = [3.142, 2.62, 3.93, 6.28, 3.93, 6.28] # Max speed

stand_pose = SE3(0, 0, 0.6)# * SE3.OA([0, 1, 0], [0, 0, 1])

def DrawRobot(traj: rtb_tools_trajectory.Trajectory):
    if isinstance(traj, rtb_tools_trajectory.Trajectory):
        JsonString = list(range(len(traj.q)))
        # JsonString = np.zeros(shape=[len(traj.q)], dtype=str)
        for i in range(len(traj.q)): # Transforming

            JsonPoints["action"] = 10
            JsonPoints["ckStatus"] = "63"
            JsonPoints["delay"] = "0.000"
            JsonPoints["insertedIndex"] = i
            JsonPoints["points"] = [
                {
                    "pointName": "",
                    "pos": {
                        "m0": str(round(degrees(traj.q[i][0]), 3)),
                        "m1": str(round(degrees(traj.q[i][1]), 3)),
                        "m2": str(round(degrees(traj.q[i][2]), 3)),
                        "m3": str(round(degrees(traj.q[i][3]), 3)),
                        "m4": str(round(degrees(traj.q[i][4]), 3)),
                        "m5": str(round(degrees(traj.q[i][5]), 3))
                    }
                }
            ]
            JsonPoints["smooth"] = 0
            JsonPoints["speed"] = "100"
            JsonPoints["toolCoord"] = 262146
            
            JsonString[i] = json.dumps(JsonPoints)

            robot.q = traj.q[i]
            env.step(dt)
    elif isinstance(traj, np.ndarray):
        for i in traj:
            robot.q = i
            env.step(dt)
    with open("./test.json", "a") as f:
        f.write("[")
        for i in range(len(JsonString)):
            if i != len(JsonString) - 1 :
                f.write(JsonString[i] + ",")
            else: f.write(JsonString[i])
        f.write(']')

def refresh():
    threading.Timer(dt, refresh).start()
    env.step(dt)

def StartSwift():
    env.launch(realtime=True)
    stand = sg.Mesh(filename=path.abspath("MERA5002_description/meshes/base_stand.stl"), pose = stand_pose)
    table = sg.Mesh(filename=path.abspath("MERA5002_description/meshes/table.stl"), pose = SE3(0.6, 1, 0.8) * SE3.OA([0, 0, 1], [-1, 0, 0]))
    stand.attach(robot)
    env.add(table)
    env.add(stand)
    env.add(robot)
    # env.add(swift.Slider(MoveGripper, min=0, max=0.03, step=0.001, desc="Gripper"))
    refresh()

def JointJog(q: ArrayLike, vel):
    # <костыль>
    viapoints = np.ndarray(shape = (2,6), dtype = float)
    viapoints[0, :] = robot.q
    viapoints[1, :] = q
    qm = rtb.mstraj(viapoints, dt, STacc, vel)
    vel_vec = qm.t
    # </костыль>
    qTraj = rtb.jtraj(robot.q, q, vel_vec)
    return qTraj

def CartesianTrajectory(start: SE3, stop: SE3, vel):
    start_point = np.ndarray(shape=[1, 3])
    start_point[0,0] = start.x
    start_point[0,1] = start.y
    start_point[0,2] = start.z

    stop_point = np.ndarray(shape=[1, 3])
    stop_point[0,0] = stop.x
    stop_point[0,1] = stop.y
    stop_point[0,2] = stop.z

    destVec = stop_point - start_point

    req_vel = rtb.p_servo(robot.fkine(robot.q), stop, vel)[0]
    
    print("Required velocity: ", req_vel)

    req_vel_norm = np.linalg.norm(req_vel[0:3])

    print("Required velocity norm: ", req_vel_norm)

    destVec_norm = np.linalg.norm(destVec)

    print("Distance: ", destVec_norm)

    travel_time = destVec_norm / req_vel_norm
    step_num = np.clip(travel_time / req_vel_norm, a_min = 25, a_max = None)
    # step_num = travel_time / req_vel_norm
    step = travel_time / step_num

    print("travel time: " , travel_time)
    print("Number of steps: ", step_num)
    print("Step: ", step)

    time_vec = np.arange(0, travel_time, step)
    # print("Time vector: ", time_vec)

    CarTraj = rtb.ctraj(start, stop, time_vec)

    sol = np.ndarray(shape = (len(CarTraj), len(robot.links) - 1))
    for i in range(len(CarTraj)):
        tep = robot.ik_LM(CarTraj[i])[0]
        sol[i, :] = tep
    # TS = np.zeros(shape=len(sol)-1)
    # TS.fill(step)
    qm = rtb.mstraj(sol, dt, tacc=0, qdmax=vel) # tsegment=TS) qdmax=vel)
    print("Arrival time: ", qm.arrive)
    return qm

# def CartesianTrajectory(start: SE3, stop: SE3, vel):
#     Tc = rtb.ctraj(start, stop, 100) # Cartesian trajectory
#     sol = np.ndarray(shape = (len(Tc), len(robot.links) - 1))
#     for i in range(len(Tc)):
#         tep = robot.ik_LM(Tc[i])[0]
#         sol[i, :] = tep
#     qm = rtb.mstraj(sol, dt, STacc, vel)
#     print(qm.arrive)
#     return qm

def MultiSegmentTrajectory(start: SE3, stop: SE3, vel):
    sol_start = robot.ik_LM(start)
    sol_stop = robot.ik_LM(stop)
    viapoints = np.ndarray(shape = (2,6), dtype = float)
    viapoints[0, :] = sol_start[0]
    viapoints[1, :] = sol_stop[0]
    qm = rtb.mstraj(viapoints, dt, STacc, vel)
    print(qm.arrive)
    return qm

def JointTrajectory(start: SE3, stop: SE3, vel):
    sol_start = robot.ik_LM(start)[0]
    sol_stop = robot.ik_LM(stop)[0]
    # <костыль>
    viapoints = np.ndarray(shape = (2,6), dtype = float)
    viapoints[0, :] = sol_start[0]
    viapoints[1, :] = sol_stop[0]
    qm = rtb.mstraj(viapoints, dt, STacc, vel)
    vel_vec = qm.t
    # </костыль>
    qj = rtb.jtraj(sol_start, sol_stop, vel_vec)
    # print(qj.t[len(qj.t) - 1])
    return qj

# def MoveGripper(q):
#     gripper.q = [q, q]

def SwiftVisualise(traj, vis_step = None, visualise_traj = False):
    if traj == None:
        robot.q = robot.qz
        ax = sg.Axes(0.3, pose = robot.fkine(robot.q) + stand_pose)
        env.add(ax)
        # for i in robot.fkine_all(robot.q):
        #     link_ax = sg.Axes(0.3, pose = i + stand_pose)
        #     env.add(link_ax)

    else:
        if visualise_traj == True:
            for i in range(0, len(traj.q), vis_step): # Visualising trajectory
                goal_ax = sg.Axes(0.05, pose = robot.fkine(traj.q[i]) + stand_pose)
                env.add(goal_ax)
        Draw = threading.Thread(target=DrawRobot, args=[traj])
        Draw.start()
        Draw.join()