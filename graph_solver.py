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

DRONE_MAX_SPEED = 1

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
    states = []
    for distance in range(0, starting_dist, 0.1):
        for speed in range(0, DRONE_MAX_SPEED, 0.1):
            states.append((distance, speed))

    # transition probability based on delta distance vs speed
    T = []
    for curr_state in states:
        for next_state in states:
            delta_dist = next_state.distance - curr_state.distance
            prob = 1 if delta_dist == curr_state.speed else 0
    
    T = # transition probability
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
                
        


    
    
