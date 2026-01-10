import csv
import itertools
import torch
from nav_msgs.msg import OccupancyGrid
from mppi_config import * 
from simulator import *


torch.manual_seed (42)



DT_OPTIONS = [0.4]

COV_X_OPTIONS = [1e-2,1e-1,1.0]
COV_Z_OPTIONS = [1e-2,1e-1,1.0]
LAMBDA_OPTIONS = [1e-2,1e-1,1.0,10,100]
GOAL_WEIGHT_OPTIONS = [100,1000,10000,100000]
ACTION_I_AXIS_WEIGHT_OPTIONS = [0,100,1000,10000, 500000]
ACTION_T_AXIS_WEIGHT_OPTIONS = [0,100,1000,10000, 500000]
STEERING_WEIGHT_OPTIONS = [0, 100,1000,10000]
HEADING_WEIGHT_OPTIONS = [0, 100,1000,10000]


# DT_OPTIONS = [0.4]

# COV_X_OPTIONS = [1e-2]
# COV_Z_OPTIONS = [1e-2]
# LAMBDA_OPTIONS = [1e-2,1e-1]
# GOAL_WEIGHT_OPTIONS = [100]
# ACTION_I_AXIS_WEIGHT_OPTIONS = [0]
# ACTION_T_AXIS_WEIGHT_OPTIONS = [0]
# STEERING_WEIGHT_OPTIONS = [0]
# HEADING_WEIGHT_OPTIONS = [0]

# HORIZON_OPTIONS = [HORIZON_LENGTH]
# DT_OPTIONS = [DT]
# NUM_SAMPLE_OPTIONS = [NUM_SAMPLES]
# COV_X_OPTIONS = [5.0]
# COV_Z_OPTIONS = [5.0]
# W_ACTION_SEQ_COST_OPTIONS = [10]
# LAMBDA_OPTIONS = [1e-2]
# GOAL_WEIGHT_OPTIONS = [1000]
# TERMINAL_GOAL_WEIGHT_OPTIONS = [1000]


def make_dummy_costmap(width=60, height=60, resolution=0.1) -> OccupancyGrid:
	msg = OccupancyGrid()
	msg.info.resolution = resolution
	msg.info.width = width
	msg.info.height = height
	msg.info.origin.position.x = -width * resolution / 2.0
	msg.info.origin.position.y = -height * resolution / 2.0
	msg.info.origin.orientation.w = 1.0
	msg.data = [0] * (width * height)
	return msg


def run_trial(params: dict) -> float:
	controller = SMMPPIController(
		STATIC_OBSTACLES,
		device=torch.device("cpu"),
		horizon_length=params["horizon_length"],
		dt=params["dt"],
		num_samples=params["num_samples"],
		cov_x=params["cov_x"],
		cov_z=params["cov_z"],
		w_action_seq_cost=params["w_action_seq_cost"],
		lambda_=params["lambda"],
		# goal_weight=params["goal_weight"],
		# terminal_goal_weight=params["terminal_goal_weight"],
	)

	controller.set_goal(torch.tensor([3.0, 0.0], dtype=torch.float32))
	controller.set_local_costmap(make_dummy_costmap())

	state = torch.zeros(3, dtype=torch.float32)
	prev_state = torch.zeros(3, dtype=torch.float32)
	robot_velocity = torch.zeros(3, dtype=torch.float32)
	empty_agents = {}

	costs = None
	for _ in range(10):
		_, _, costs, _ = controller.compute_control(
			current_state=state,
			previous_robot_state=prev_state,
			robot_velocity=robot_velocity,
			agent_states=empty_agents,
			previous_agent_states=empty_agents,
			agent_velocities=empty_agents,
		)

	return float(costs.mean().item()) if costs is not None else float("nan")


def sweep_and_save(output_path: str = "param_sweep_results.csv") -> None:
	header = [
		"dt",
		"cov_x",
		"cov_z",
		"lambda",
		"goal_weight",
		"action_i_axis_weight",
		"action_t_axis_weight",
		"steering_weight",
		"heading_weight",
		"final_dist",
		"termination",
		"time_total",
	]

	rows = []
	done = 0
	total = len(DT_OPTIONS) * len(COV_X_OPTIONS) * len(COV_Z_OPTIONS) * len(LAMBDA_OPTIONS) * len(GOAL_WEIGHT_OPTIONS) * len(ACTION_I_AXIS_WEIGHT_OPTIONS) * len(ACTION_T_AXIS_WEIGHT_OPTIONS) * len(STEERING_WEIGHT_OPTIONS) * len(HEADING_WEIGHT_OPTIONS)
	for combo in itertools.product(
		DT_OPTIONS,
		COV_X_OPTIONS,
		COV_Z_OPTIONS,
		LAMBDA_OPTIONS,
		GOAL_WEIGHT_OPTIONS,
		ACTION_I_AXIS_WEIGHT_OPTIONS,
		ACTION_T_AXIS_WEIGHT_OPTIONS,
		STEERING_WEIGHT_OPTIONS,
		HEADING_WEIGHT_OPTIONS,
	):
		params = {
			"dt": combo[0],
			"cov_x": combo[1],
			"cov_z": combo[2],
			"lambda": combo[3],
			"goal_weight": combo[4],
			"action_i_axis_weight": combo[5],
			"action_t_axis_weight": combo[6],
			"steering_weight": combo[7],
			"heading_weight": combo[8],
		}

		final_dist, termination, time_total = run_simulator(params)
		row = {**params, "final_dist": final_dist, "termination": termination, "time_total": time_total}
		rows.append(row)

		done += 1
		print(f"[param sweep] {done}/{total} completed ({done/total:.2%})")
		if termination:
			print ("Stopping sweep due to successful termination.")
			break

	with open(output_path, "w", newline="") as csvfile:
		writer = csv.DictWriter(csvfile, fieldnames=header)
		writer.writeheader()
		writer.writerows(rows)


if __name__ == "__main__":
	sweep_and_save()
