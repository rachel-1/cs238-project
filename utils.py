from scipy import stats
import math
import numpy as np
from visualization import *
import time
import copy

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

def get_candidate_paths(path, policy, G, global_time, min_value):
    # Get n-1 candidate paths, where n is length of best_path
    # Prevent any ONE of the same connections of the best path being used to
    # generate candidates
    candidates = set()
    for i in range(len(path)-1):
        H = copy.deepcopy(G)
        H.remove_edge(path[i], path[i+1])
        heuristic = lambda n0, n1: calc_dist(H.nodes[n0], H.nodes[n1]) # for A*
        try: # first check if a path exists from current to end even without removed edge
            candidate_path = nx.astar_path(H, 'current', 'end', heuristic=heuristic)
        except nx.exception.NetworkXNoPath as err:
            continue

        try: # now use equation 9 to reject edges
            for j in range(len(candidate_path)-1):
                n0 = candidate_path[j]
                n1 = candidate_path[j+1]
                speed = H.nodes[n0]['speed']
                distance = calc_dist(H.nodes[n0], H.nodes[n1])
                state_idx = policy.state_to_idx((speed, distance, global_time+j)) # TODO - this line bugs out
                if policy.value_func[state_idx] < min_value:
                    continue
            candidates.add(candidate_path)
        except:
            continue
        del H # free memory
    return candidates

def run_policy(policy, G, next_node, global_time, min_value, display=True):
    target_pos = (G.nodes[next_node]['x'], G.nodes[next_node]['y'])
    speed = G.nodes['current']['speed']
    distance = calc_dist(G.nodes['current'], G.nodes[next_node])

    num_steps = 0
    total_accel = 0
    while True:
        # constrained flight
        if 'arrival_time' in G.nodes[next_node]:
            time_remaining = G.nodes[next_node]['arrival_time'].mean - global_time
            # check whether to abort
            if hasattr(policy, 'value_func'): # check if constrained flight
                current_state_idx = policy.state_to_idx((speed, distance, global_time))
                if policy.value_func[current_state_idx] < min_value:
                    return True, False, num_steps, total_accel

            if time_remaining < 0: return False, False, num_steps, total_accel
            # overwrite time remaining in the state
            # TODO - if the bus is super late (> mean + 2*std_dev)
            speed, distance = policy.policy_step(speed, distance, time_remaining)
        else:
            time_remaining = None
            speed, distance = policy.policy_step(speed, distance)
        print("speed, distance: ", speed, distance) # TODO - remove debug statement

        total_accel += abs(speed - G.nodes['current']['speed'])

        current_pos = (G.nodes['current']['x'], G.nodes['current']['y'])

        # update x,y position of drone
        if num_steps != 0:
            x, y = calc_new_coord(current_pos, target_pos, distance)
            G.nodes['current']['speed'] = speed
            G.nodes['current']['x'] = x
            G.nodes['current']['y'] = y
        print("G.nodes['current']: ", G.nodes['current']) # TODO - remove debug statement

        # if we have reached goal (speed=0, dist=0, remaining_time=0 if present)
        if speed == 0 and distance == 0 \
           and (time_remaining is None or time_remaining == 0):
            return False, True, num_steps, total_accel
        if display: display_graph(G)
        num_steps += 1
        global_time += 1
        if display: time.sleep(0.5)


    # not aborted, failed to reach goal
    return False, False, num_steps, total_accel

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
