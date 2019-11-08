import networkx as nx
from graph_setup import init_graph
import math
import matplotlib.pyplot as plt

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

DRONE_MAX_ACCEL = 1
DRONE_MAX_SPEED = 1
DRONE_MAX_DIST = 150
ACCEL_STEP = 0.1
SPEED_STEP = 0.1
DIST_STEP = 0.1

DIST_VARIANCE = 1
SPEED_VARIANCE = 1

G = init_graph('test1')

global_time = 0 # in secs?

# drone moves 1 unit per sec
# bus moves 2 units per sec

def display_graph(G):
    pos = nx.random_layout(G)
    #pos = nx.spring_layout(G) # positions for all nodes
    nx.draw_networkx_nodes(G,pos)
    nx.draw_networkx_edges(G,pos,arrowstyle='->')
    nx.draw_networkx_labels(G,pos)

    plt.show()
    #plt.savefig('graph.png', dpi=200)

def calc_dist(nodeOne, nodeTwo):
    return math.sqrt((nodeOne['x'] - nodeTwo['x'])**2
              + (nodeOne['y'] - nodeTwo['y'])**2)

def calc_edge_weights(G):
    import mdptoolbox, mdptoolbox.example
    # (A, S, S)
    # state: (distance from goal, speed)
    # states[r,c]:
    #  - speed = r * SPEED_STEP
    #  - distance from goal = c * DIST_STEP
    num_actions = DRONE_MAX_ACCEL // ACCEL_STEP
    num_rows = DRONE_MAX_SPEED // SPEED_STEP
    num_cols = DRONE_MAX_DIST // DIST_STEP
    num_states = num_rows * num_cols
    # transition probability based on delta distance vs speed
    T = np.zeros((num_actions,num_states,num_states))
    for action_index, intended_action in enumerate(range(0, DRONE_MAX_ACCEL, ACCEL_STEP)):
        for curr_state_r in range(num_rows):
            for curr_state_c in range(num_cols):
                curr_state_index = np.ravel_multi_index((curr_state_r), (curr_state_c)) #r*num_cols + c
                curr_state_speed = curr_state_r * SPEED_STEP
                curr_state_distance = curr_state_c * DIST_STEP
                intended_speed = scipy.stats.norm(curr_state_speed + intended_action, SPEED_VARIANCE)
                #states = np.zeros((num_rows, num_cols))
                # calculate next state
                for next_state_r in range(num_rows):
                    current_speed = next_state_r * SPEED_STEP
                    # probability of getting speed (as represented by row in states table)
                    speed_prob = intended_speed.cdf(current_speed + SPEED_STEP/2) - intended_speed.cdf(current_speed - SPEED_STEP/2)
                    intended_dist = scipy.states.norm(curr_state_distance - intended_speed, DIST_VARIANCE)
                    for next_state_c in range(num_cols):
                        current_dist = next_state_c * DIST_STEP
                        total_prob = intended_dist.cdf(current_dist + DIST_STEP/2) - intended_dist.cdf(current_dist - DIST_STEP/2)
                        #states[r,c] = total_prob
                        next_state_index = np.ravel_multi_index((next_state_r), (next_state_c)) #r*num_cols + c
                        T[action_index, curr_state_index, next_state_index] = total_prob

    
    R = # reward matrix
    discount = 0.98
    vi = mdptoolbox.mdp.ValueIteration(T, R, discount)
    vi.run()
    print(vi.policy)

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
display_graph(G)
path = nx.astar_path(G, 'current', 'end')
print(path)
                
        


    
    
