import numpy as np
from config import *
import matplotlib.pyplot as plt
import networkx as nx
from utils import *

def visualize_probabilities(T):
    plt.imshow(T, cmap='hot', interpolation='nearest')
    plt.show()
    
def display_graph(G):
    pos = nx.random_layout(G)
    nx.draw_networkx_nodes(G,pos)
    nx.draw_networkx_edges(G,pos,arrowstyle='->')
    nx.draw_networkx_labels(G,pos)

    plt.show()
    #plt.savefig('graph.png', dpi=200)

def visualize_T(T):
    for action_index, intended_action in enumerate(np.arange(-DRONE_MAX_ACCEL, DRONE_MAX_ACCEL+ACCEL_STEP, ACCEL_STEP)):
        print("action",intended_action)
        num_rows = int(DRONE_MAX_SPEED/SPEED_STEP + 1)
        num_cols = int(DRONE_MAX_DIST/DIST_STEP + 1)
        for curr_state_r in range(num_rows):
            for curr_state_c in range(num_cols):
                curr_state_index = curr_state_r*num_cols + curr_state_c
                curr_state_speed = curr_state_r * SPEED_STEP
                curr_state_distance = curr_state_c * DIST_STEP
                print("({}, {}): ".format(curr_state_speed, curr_state_distance), end='')
                print(T[action_index, curr_state_index])

def simulate_policy(pi, T, R):
    timesteps = pi.shape[1]
    state_idx = 1
    for i in range(timesteps):
        action_idx = pi[state_idx, i]
        new_state_idx = np.argmax(T[action_idx, state_idx])
        reward = R[state_idx, action_idx]
        print("from s={}, a={} yields r={} and s'={}".format(
            idx_to_state(state_idx),
            idx_to_action(action_idx),
            reward,
            idx_to_state(new_state_idx)))
        state_idx = new_state_idx

