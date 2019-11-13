from config import *

num_rows = int(DRONE_MAX_SPEED/SPEED_STEP + 1)
num_cols = int(DRONE_MAX_DIST/DIST_STEP + 1)

def idx_to_action(action_idx):
    return -DRONE_MAX_ACCEL + action_idx*ACCEL_STEP

def action_to_idx(action):
    return int((action - (-DRONE_MAX_ACCEL))/ACCEL_STEP)

def idx_to_state(state_idx):
    curr_state_c = state_idx % num_cols
    curr_state_r = int(state_idx / num_cols)
    return (curr_state_r * SPEED_STEP, curr_state_c * DIST_STEP)

def state_to_idx(state):
    speed, distance = state
    curr_state_r = int(speed / SPEED_STEP)
    curr_state_c = int(distance / DIST_STEP)
    return curr_state_r*num_cols + curr_state_c
