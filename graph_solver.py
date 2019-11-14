import networkx as nx
import numpy as np
from graph_setup import init_graph
from utils import *
from visualization import *
from config import *

def calc_T(num_actions, num_rows, num_cols):
    num_states = num_rows * num_cols
    # transition probability based on delta distance vs speed
    T = np.zeros((num_actions,num_states,num_states))
    for action_index, intended_action in enumerate(np.arange(-DRONE_MAX_ACCEL, DRONE_MAX_ACCEL+ACCEL_STEP, ACCEL_STEP)):
        for curr_state_r in range(num_rows):
            for curr_state_c in range(num_cols):
                curr_state_index = curr_state_r*num_cols + curr_state_c
                curr_state_speed = curr_state_r * SPEED_STEP
                curr_state_distance = curr_state_c * DIST_STEP
                intended_speed = RandVar(curr_state_speed + intended_action, SPEED_VARIANCE)
                #print('-'*80)
                # calculate next state
                for next_state_r in range(num_rows):
                    current_speed = next_state_r * SPEED_STEP # can't go backwards
                    # probability of getting speed (as represented by row in states table)
                    speed_prob = intended_speed.probability(current_speed, SPEED_STEP/2)
                    intended_dist = RandVar(abs(curr_state_distance - intended_speed.mean), DIST_VARIANCE)
                    for next_state_c in range(num_cols):
                        current_dist = next_state_c * DIST_STEP
                        total_prob = speed_prob*intended_dist.probability(current_dist, DIST_STEP/2) # TODO - independent??
                        #print("a={}: ({}, {}) -> ({}, {}) = {}".format(intended_action, curr_state_speed, curr_state_distance, current_speed, current_dist, total_prob))
                        next_state_index = next_state_r*num_cols + next_state_c
                        T[action_index, curr_state_index, next_state_index] = total_prob

                # ensure T is valid (every row must add up to 1)
                if T[action_index, curr_state_index].sum() == 0:
                    T[action_index, curr_state_index, curr_state_index] = 1
                T[action_index, curr_state_index] /= np.linalg.norm(T[action_index, curr_state_index])
    # round to allow values to add up to exactly 1
    T = np.around(T, decimals=5)
    #visualize_T(T)
    return T

def calc_edge_weights(G):

    #riding = current_node['riding']
    for node, neighbor in G.edges():
        current_node = G.nodes()[node]
        speed = current_node.get('speed', 0)
        neighbor_node = G.nodes()[neighbor]
        distance = calc_dist(current_node, neighbor_node)

        # riding edge
        if node != 'current' and neighbor != 'end':
            weight = distance/BUS_SPEED
        else:
            # unconstrained flight
            if neighbor == 'end':
                horizon = 'inf'
            # constrained flight
            else:
                horizon = neighbor_node['estimated_arrival_time'] - global_time
            weight = calc_edge_weight((speed, distance), horizon)
            
        G.add_edge(node, neighbor, weight=weight)

def calc_edge_weight(start_state, horizon, discount=0.98):
    import mdptoolbox, mdptoolbox.example
    # (A, S, S)
    # state: (distance from goal, speed)
    # states[r,c]:
    #  - speed = r * SPEED_STEP
    #  - distance from goal = c * DIST_STEP

    num_actions = int(2*DRONE_MAX_ACCEL/ACCEL_STEP + 1)
    num_rows = int(DRONE_MAX_SPEED/SPEED_STEP + 1)
    num_cols = int(start_state[1]/DIST_STEP + 1)
    num_states = num_rows * num_cols
    T = calc_T(num_actions, num_rows, num_cols)
    
    # penalty for acceleration (S, A)
    R = np.zeros((num_states, num_actions))
    R[:] = -abs(np.arange(-DRONE_MAX_ACCEL, DRONE_MAX_ACCEL+ACCEL_STEP, ACCEL_STEP))*ACCEL_PENALTY
    
    final_rewards = np.ones(num_states)*-10
    final_rewards[0] = 0

    if horizon != 'inf':
        vi = mdptoolbox.mdp.FiniteHorizon(T, R, discount, N=horizon, h=final_rewards)
    else:
        # penalize distance to goal - TODO
        R = R.T
        R = R.reshape((num_actions, num_rows, num_cols))
        # TODO
        R[:, :] -= 5*np.arange(0, start_state[1]+DIST_STEP, DIST_STEP)
        R = R.reshape(num_actions, num_states)
        R = R.T
        # TODO - how to encourage agent to reach goal?
        vi = mdptoolbox.mdp.ValueIteration(T, R, discount=1)
    vi.run()
    start_state_idx = state_to_idx(start_state, start_state[1])
    simulate_policy(vi.policy, T, R, start_state_idx, start_state[1])
    if horizon != 'inf':
        return -1*vi.V[start_state_idx, 0]
    else:
        return -1*vi.V[start_state_idx]

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


G = init_graph('test1')
global_time = 0 # in secs?
add_edges(G)
calc_edge_weights(G)
for edge in G.edges().data():
    print("edge: ", edge) # TODO - remove debug statement
display_graph(G)
path = nx.astar_path(G, 'current', 'end')
print(path)
# now need to cache and then follow policy


    
    
