import os
import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np

print("Press ctrl+\ to quit process.")

def load_desired_trajectory(filename):
        trajectory_dir = os.path.join(os.path.dirname(__file__), "trajectories")
        file_path = os.path.join(trajectory_dir, f"{filename}.npz")
        if os.path.exists(file_path):
            data = np.load(file_path)
            print(f"[INFO] Loaded trajectory from {trajectory_dir}/{filename}.npz")
            return data["t"], data["q"], data["q_dot"], data["tau"]
        else:
            return None, None, None, None
        
t_interp, q_interp, q_dot_interp, tau_interp = load_desired_trajectory("nominal_trajectory")
dt = t_interp[1]-t_interp[0]

np.set_printoptions(precision=3, suppress=True)
arm = unitree_arm_interface.ArmInterface(hasGripper=True)
armModel = arm._ctrlComp.armModel
arm.setFsmLowcmd()

duration = len(t_interp)
q_init = q_interp[:,0]
q_end = q_interp[:,-1]

warm_up_time = 5
start_time = time.time()

while time.time() - start_time < warm_up_time:
    arm.q = q_interp[:,0]
    arm.qd = q_dot_interp[:,0]
    arm.tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6)) # set torque
    arm.gripperQ = 0

    arm.setArmCmd(arm.q, arm.qd, arm.tau)
    arm.setGripperCmd(arm.gripperQ, arm.gripperQd, arm.gripperTau)
    arm.sendRecv()# udp connection
    # print(arm.lowstate.getQ())
    time.sleep(dt)

for i in range(0, duration):
    arm.q = q_interp[:,i]
    arm.qd = q_dot_interp[:,i]
    arm.tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6)) # set torque
    arm.gripperQ = 0

    arm.setArmCmd(arm.q, arm.qd, arm.tau)
    arm.setGripperCmd(arm.gripperQ, arm.gripperQd, arm.gripperTau)
    arm.sendRecv()# udp connection
    # print(arm.lowstate.getQ())
    time.sleep(dt)

finish_time = time.time()
holding_time = 5
while time.time() - finish_time < holding_time:
    arm.q = q_interp[:,-1]
    arm.qd = np.array([0,0,0,0,0,0])
    arm.tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6)) # set torque
    arm.gripperQ = 0

    arm.setArmCmd(arm.q, arm.qd, arm.tau)
    arm.setGripperCmd(arm.gripperQ, arm.gripperQd, arm.gripperTau)
    arm.sendRecv()# udp connection
    # print(arm.lowstate.getQ())
    time.sleep(dt)

arm.loopOn()
arm.backToStart()
arm.loopOff()
