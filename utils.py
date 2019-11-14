from config import *
from scipy import stats
import math

def calc_dist(nodeOne, nodeTwo):
    return math.sqrt((nodeOne['x'] - nodeTwo['x'])**2
              + (nodeOne['y'] - nodeTwo['y'])**2)

class RandVar():
    def __init__(self, mean, variance):
        self.mean = mean
        self.variance = variance
        if variance != 0:
            self.distribution = stats.norm(mean, variance)

    def probability(self, value, delta): # TODO
        """
        Probability of variable taking on value in range [lower, upper]
        """
        if self.variance != 0:
            return self.distribution.cdf(value + delta) - self.distribution.cdf(value - delta)
        return 1 if self.mean == value else 0

num_rows = int(DRONE_MAX_SPEED/SPEED_STEP + 1)

def idx_to_action(action_idx):
    return -DRONE_MAX_ACCEL + action_idx*ACCEL_STEP

def action_to_idx(action):
    return int((action - (-DRONE_MAX_ACCEL))/ACCEL_STEP)

def idx_to_state(state_idx, max_dist):
    num_cols = int(max_dist/DIST_STEP + 1)
    curr_state_c = state_idx % num_cols
    curr_state_r = int(state_idx / num_cols)
    return (curr_state_r * SPEED_STEP, curr_state_c * DIST_STEP)

def state_to_idx(state, max_dist):
    num_cols = int(max_dist/DIST_STEP + 1)
    speed, distance = state
    curr_state_r = int(speed / SPEED_STEP)
    curr_state_c = int(distance / DIST_STEP)
    return curr_state_r*num_cols + curr_state_c
