import numpy as np
import torch
from datetime import datetime

NOW = datetime.now()

VMAX = 1.4 #0.7
DT = 0.4
HORIZON_LENGTH =  20 #16
MAX_AGENT_NUM = 14
ACTIVE_AGENTS = 1
AGENT_GOALS = np.array([[0.0,-2.35], [0.0, 2.75]]) #goal of the agents, only required for evaluation metrics
HZ = 0.05 #frequency at which the controller should run
ANCA_CONTROL_POINTS = 5
USE_TERMINAL_COST = True
ANGULAR_THRESHOLD = 0.3
STATIC_OBSTACLES = [
            [(-0.7, -2.0), (-2.0, -2.0), (-2.0, 2.0), (-0.7, 2.0)],  
            [(0.7, -2.0), (2.0, -2.0), (2.0, 2.0), (0.7, 2.0)]  
            ]  # polgon shape of the static obstacles

NUM_SAMPLES = 500
NEED_ODOM = False
NEED_LASER = False
HUMAN_FRAME = "human" # for the TF transform from the motion capture
RADIUS = 0.2
NUM_CYCLES = 10 # Number of time to cycle between the goals
# GOALS = np.array([[0.0, -1.5],[0.0, 1.5]])
GOALS = np.array([[0.0,-1.5],[0.0, 2.0]])
# GOALS = np.array([[1.0,0.0],[-2.0, 0.0]])
REPEAT_GOALS = True
TERMINATION_TOLERANCE = 0.4

ACKNOWLEDGE_RADIUS = 2.0
SIGMA_H = 0.3
SIGMA_S = 0.3
SIGMA_R = 0.6

Q_OBS = 10e3 #Use with terminal cost