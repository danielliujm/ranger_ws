import csv
import itertools
import torch
from nav_msgs.msg import OccupancyGrid
from alpaca_navigation.mppi_config import (
	STATIC_OBSTACLES,
	HORIZON_LENGTH,
	DT,
	NUM_SAMPLES,
)
from sm_mppi_for_param_sweep import SMMPPIController


HORIZON_OPTIONS = [10,15,20,25,30,35,40]
DT_OPTIONS = [0.1,0.2,0.3,0.4,0.5]
NUM_SAMPLE_OPTIONS = [250,400,500]
COV_X_OPTIONS = [1e-2,1e-1,1.0,2,3,4,5,6,7,8,9,10]
COV_Z_OPTIONS = [1e-2,1e-1,1.0,2,3,4,5,6,7,8,9,10]
W_ACTION_SEQ_COST_OPTIONS = [1e-3,1e-2,1e-1,1.0,1e+2,1e+3]
LAMBDA_OPTIONS = [1e-3,1e-2,1e-1,1.0,10,100,1000]
GOAL_WEIGHT_OPTIONS = [1e-3,1e-2,1e-1,1.0,10,100,1000]
TERMINAL_GOAL_WEIGHT_OPTIONS = [1e-3,1e-2,1e-1,1.0,10,100,1000]

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
		"horizon_length",
		"dt",
		"num_samples",
		"cov_x",
		"cov_z",
		"w_action_seq_cost",
		"lambda",
		"final_cost",
	]

	rows = []
	total = len(HORIZON_OPTIONS) * len(DT_OPTIONS) * len(NUM_SAMPLE_OPTIONS) * len(COV_X_OPTIONS) * len(COV_Z_OPTIONS) * len(W_ACTION_SEQ_COST_OPTIONS) * len(LAMBDA_OPTIONS)
	done = 0
	for combo in itertools.product(
		HORIZON_OPTIONS,
		DT_OPTIONS,
		NUM_SAMPLE_OPTIONS,
		COV_X_OPTIONS,
		COV_Z_OPTIONS,
		W_ACTION_SEQ_COST_OPTIONS,
		LAMBDA_OPTIONS,
		# GOAL_WEIGHT_OPTIONS,
		# TERMINAL_GOAL_WEIGHT_OPTIONS,
	):
		params = {
			"horizon_length": combo[0],
			"dt": combo[1],
			"num_samples": combo[2],
			"cov_x": combo[3],
			"cov_z": combo[4],
			"w_action_seq_cost": combo[5],
			"lambda": combo[6],
			# "goal_weight": combo[7],
			# "terminal_goal_weight": combo[8],
		}

		final_cost = run_trial(params)
		row = {**params, "final_cost": final_cost}
		rows.append(row)

		done += 1
		print(f"[param sweep] {done}/{total} completed ({done/total:.2%})")

	with open(output_path, "w", newline="") as csvfile:
		writer = csv.DictWriter(csvfile, fieldnames=header)
		writer.writeheader()
		writer.writerows(rows)


if __name__ == "__main__":
	sweep_and_save()
