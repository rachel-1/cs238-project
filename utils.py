from scipy import stats
import math
import numpy as np
from visualization import *
import time

def calc_dist(n_one, n_two):
    dist = math.sqrt((n_one['x'] - n_two['x'])**2 + (n_one['y'] - n_two['y'])**2)
    return int(dist/DIST_STEP)*DIST_STEP

def calc_new_coord(prev_pos, target_pos, newDist):
    if newDist == 0: return target_pos
    oldDist = math.sqrt((prev_pos[0] - target_pos[0])**2 + (prev_pos[1] - target_pos[1])**2)
    if oldDist == 0: return prev_pos # TODO - ignores overshooting
    newHeight = ((newDist)*abs(prev_pos[1] - target_pos[1]))/oldDist
    newWidth = math.sqrt((newDist)**2 - (newHeight)**2)
    target_pos = np.array(target_pos)
    sign = np.sign(np.array(prev_pos) - target_pos)
    new_pos = target_pos + sign*np.array([newWidth, newHeight])
    return new_pos[0], new_pos[1]

def run_policy(mdp, G, next_node, global_time, display=True):
    target_pos = (G.nodes[next_node]['x'], G.nodes[next_node]['y'])
    state = mdp.start_state
    print("state: ", state) # TODO - remove debug statement
    num_steps = 0
    while True:
        # constrained flight
        if hasattr(mdp, 'num_timesteps'):
            time_remaining = G.nodes[next_node]['arrival_time'].mean - global_time
            print("time_remaining: ", time_remaining) # TODO - remove debug statement
            if time_remaining < 0: return False, num_steps
            state = mdp.policy_step(state)
        else:
            if 'arrival_time' in G.nodes[next_node]:
                time_remaining = G.nodes[next_node]['arrival_time'].mean - global_time
                print("time_remaining: ", time_remaining) # TODO - remove debug statement
                state = mdp.policy_step(state, time_remaining)
            else:
                state = mdp.policy_step(state)
        print("state: ", state) # TODO - remove debug statement

        current_pos = (G.nodes['current']['x'], G.nodes['current']['y'])

        # update x,y position of drone
        if num_steps != 0:
            x, y = calc_new_coord(current_pos, target_pos, state[1])
            G.nodes['current']['speed'] = state[0]
            G.nodes['current']['x'] = x
            G.nodes['current']['y'] = y
        print("G.nodes['current']: ", G.nodes['current']) # TODO - remove debug statement

        # if we have reached goal (speed=0, dist=0, remaining_time=0 if present)
        if sum(state) == 0: return True, num_steps
        if display: display_graph(G)
        num_steps += 1
        global_time += 1
        if display: time.sleep(0.5)
        

    # failed to reach goal
    return False, num_steps

def update_estimate(prev_rand_var, global_time):
    # TODO: ensure variance isn't 0
    mean = max(0,int(prev_rand_var.distribution.rvs(1)))
    time_remaining = global_time - mean
    if time_remaining == 0:
        variance = 0
    else:
        variance = prev_rand_var.variance
        variance -= variance/time_remaining
    return RandVar(mean, variance)

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
