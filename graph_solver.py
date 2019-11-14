import networkx as nx
import numpy as np
from graph_setup import init_graph
from scipy import stats
import math
from utils import *
from visualization import *
from config import *

'''
needs access to routes available
set of all active routes + current distribution of all arrival times of all future stops
probs some graph search
implicit graph —> no edges at first, generate them on the fly
position x_s(t)
goal x_g(t)
for all trips t, route R_t
define what info each vertex should have
vertex for start, goal
each vertex should be mutable (have timestamp) except maybe start and goal
at every t, distributions of arrival time gets updated
postition and time distribution
sometimes edges go out of existence as distributions are updated
curr: doing feasibility test to see if edges ok
'''

G = init_graph('test1')

global_time = 0 # in secs?

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

def calc_edge_weights(G):
    import mdptoolbox, mdptoolbox.example
    # (A, S, S)
    # state: (distance from goal, speed)
    # states[r,c]:
    #  - speed = r * SPEED_STEP
    #  - distance from goal = c * DIST_STEP
    num_actions = int(2*DRONE_MAX_ACCEL/ACCEL_STEP + 1)
    num_rows = int(DRONE_MAX_SPEED/SPEED_STEP + 1)
    num_cols = int(DRONE_MAX_DIST/DIST_STEP + 1)
    num_states = num_rows * num_cols
    # transition probability based on delta distance vs speed
    T = np.zeros((num_actions,num_states,num_states))
    for action_index, intended_action in enumerate(np.arange(-DRONE_MAX_ACCEL, DRONE_MAX_ACCEL+ACCEL_STEP, ACCEL_STEP)):
        assert(action_index == action_to_idx(intended_action))
        assert(intended_action == idx_to_action(action_index))
        for curr_state_r in range(num_rows):
            for curr_state_c in range(num_cols):
                curr_state_index = curr_state_r*num_cols + curr_state_c
                curr_state_speed = curr_state_r * SPEED_STEP
                curr_state_distance = curr_state_c * DIST_STEP
                assert(state_to_idx((curr_state_speed, curr_state_distance)) == curr_state_index)
                assert(idx_to_state(curr_state_index) == (curr_state_speed, curr_state_distance))
                intended_speed = RandVar(curr_state_speed + intended_action, SPEED_VARIANCE)
                print('-'*80)
                # calculate next state
                for next_state_r in range(num_rows):
                    current_speed = next_state_r * SPEED_STEP # can't go backwards
                    # probability of getting speed (as represented by row in states table)
                    speed_prob = intended_speed.probability(current_speed, SPEED_STEP/2)
                    intended_dist = RandVar(abs(curr_state_distance - intended_speed.mean), DIST_VARIANCE)
                    for next_state_c in range(num_cols):
                        current_dist = next_state_c * DIST_STEP
                        total_prob = speed_prob*intended_dist.probability(current_dist, DIST_STEP/2) # TODO - independent??
                        print("a={}: ({}, {}) -> ({}, {}) = {}".format(intended_action, curr_state_speed, curr_state_distance, current_speed, current_dist, total_prob))
                        next_state_index = next_state_r*num_cols + next_state_c
                        T[action_index, curr_state_index, next_state_index] = total_prob

                # ensure T is valid (every row must add up to 1)
                if T[action_index, curr_state_index].sum() == 0:
                    T[action_index, curr_state_index, curr_state_index] = 1
                T[action_index, curr_state_index] /= np.linalg.norm(T[action_index, curr_state_index])
    # round to allow values to add up to exactly 1
    T = np.around(T, decimals=5)
    visualize_T(T)
    #visualize_probabilities(T[0])
    # penalty for acceleration (S, A)
    R = np.zeros((num_states, num_actions))
    R[:] = -abs(np.arange(-DRONE_MAX_ACCEL, DRONE_MAX_ACCEL+ACCEL_STEP, ACCEL_STEP))*ACCEL_PENALTY
    discount = 0.98
    final_rewards = np.ones(num_states)*-10
    final_rewards[0] = 0
    vi = mdptoolbox.mdp.FiniteHorizon(T, R, discount, N=3, h=final_rewards)
    vi.run()
    simulate_policy(vi.policy, T, R)

# TODO - check for cycles?!
# TODO - remove nodes that have been passed already
def add_edges(G):
    # unconstrained: edges that connect to end goal
    # riding: edges between bus stops (for a given bus) already present
    # constrained: edges between drone reachable intersection points
    for node_name, node_data in G.nodes.items():

        # don't want edges from end back to other nodes
        if node_name == 'end': continue
        
        for other_name, other_data in G.nodes.items():
            # don't make edge to self
            if node_name == other_name: continue

            # should never go backward to current
            if other_name == 'current': continue

            if other_name == 'end': # add unconstrained 
                G.add_edge(node_name, other_name)
            else: # add constrained edge
                dist_between_nodes = calc_dist(node_data, other_data)
                if node_name == 'current':
                    available_time = other_data['estimated_arrival_time'] - global_time
                else:
                    available_time = other_data['estimated_arrival_time'] - node_data['estimated_arrival_time']

                if available_time > 0 and dist_between_nodes/available_time <= DRONE_MAX_SPEED:
                    G.add_edge(node_name, other_name)

add_edges(G)
calc_edge_weights(G)
display_graph(G)
#path = nx.astar_path(G, 'current', 'end')
#print(path)
                
        


    
    
