import numpy as np
from config import *
import matplotlib.pyplot as plt
import networkx as nx
#from utils import *
import utils

def visualize_probabilities(T):
    plt.imshow(T, cmap='hot', interpolation='nearest')
    plt.show()
    
def display_graph(G):
    # Get draw positions of nodes
    x_positions = nx.get_node_attributes(G, 'x')
    y_positions = nx.get_node_attributes(G, 'y')
    xy_positions = dict()
    for node in G.nodes:
        xy_positions[node] = (x_positions[node], y_positions[node])

    fig, ax = plt.subplots()

    # Draw edge weights
    #TODO: Update with actual edge weight label once they're added
    labels = nx.get_edge_attributes(G, 'weight')
    # To test: labels = {e: str(e) for e in G.edges}
    nx.draw_networkx_edge_labels(G, pos=xy_positions, edge_labels=labels)

    # Draw rest of graph
    nx.draw_networkx(G, pos=xy_positions, ax=ax)
    ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)

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
