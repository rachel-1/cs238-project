from utils import *
from local_layer_mdp import *

def calc_edge_weights(G, global_time):
    mdps = {}
    #riding = current_node['riding']
    for node, neighbor in G.edges():
        current_node = G.nodes()[node]
        speed = current_node.get('speed', 0)
        neighbor_node = G.nodes()[neighbor]
        distance = calc_dist(current_node, neighbor_node)
        
        # riding edge
        if node != 'current' and neighbor != 'end':
            horizon = neighbor_node['estimated_arrival_time'] - global_time
            mdp = RidingPolicy((speed, distance), horizon)
            weight = distance/BUS_SPEED + get_global_edge_weight((node, neighbor), isRiding=True)
        else:
            # unconstrained edge
            if neighbor == 'end':
                mdp = UnconstrainedFlightMDP((speed,distance))
                weight = mdp.get_edge_weight() + get_global_edge_weight((node, neighbor), EdgeType.UNCONSTRAINED)
            # constrained edge
            else:
                horizon = neighbor_node['estimated_arrival_time'] - global_time
                mdp = ConstrainedFlightMDP((speed,distance), horizon)
                weight = mdp.get_edge_weight() + get_global_edge_weight((node, neighbor), EdgeType.CONSTRAINED)

            mdp.solve()

        mdps[(node, neighbor)] = mdp
        G.add_edge(node, neighbor, weight=weight)
    return mdps

TAKEOFF_COST_WEIGHT = 1 # accelerate vertically up
LANDING_COST_WEIGHT = TAKEOFF_COST_WEIGHT + 1 # decelerate vertically down + damage from floor impact
ACCEL_COST_WEIGHT = 1 # per unit of acceleration
TIME_COST_WEIGHT = 1

def get_global_edge_weight(edge, edgeType):
    time_delta = neighbor_node['estimated_arrival_time'] - global_time
    timeCost = time_delta * TIME_COST_WEIGHT
    # energy costs
    acceleration = (neighbor['speed'] - curr['speed'])/time_delta
    energyCost =  ACCEL_COST_WEIGHT * acceleration
    if edgeType == EdgeType.RIDING:
      # To ride, will land once on bus then take off
      energyCost += LANDING_COST + TAKEOFF_COST
    elif edgeType == EdgeType.UNCONSTRAINED:
      # Will land once at end
      energyCost += LANDING_COST_WEIGHT
    return -(energyCost + timeCost)

# TODO - check for cycles?!
# TODO - remove nodes that have been passed already
# TODO - skip adding edges from unreachable nodes
def add_edges(G, global_time):
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
                G.add_edge(node_name, other_name, custom=True)
            else: # add constrained edge
                dist_between_nodes = calc_dist(node_data, other_data)
                if node_name == 'current':
                    available_time = other_data['estimated_arrival_time'] - global_time
                else:
                    available_time = other_data['estimated_arrival_time'] - node_data['estimated_arrival_time']

                if available_time > 0 and dist_between_nodes/available_time <= DRONE_MAX_SPEED:
                    G.add_edge(node_name, other_name, custom=True)        



    
    
