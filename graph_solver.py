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
            horizon = neighbor_node['arrival_time'].mean - global_time
            mdp = RidingPolicy((speed, distance), horizon)
            weight = distance/BUS_SPEED
        else:
            if neighbor == 'end':
                mdp = UnconstrainedFlightMDP((speed,distance))
            else:
                horizon = neighbor_node['arrival_time'].mean - global_time
                mdp = ConstrainedFlightMDP((speed,distance), horizon)
            mdp.solve()
            weight = mdp.get_edge_weight()
        mdps[(node, neighbor)] = mdp
        G.add_edge(node, neighbor, weight=weight)
    return mdps

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
                    available_time = other_data['arrival_time'].mean - global_time
                else:
                    available_time = other_data['arrival_time'].mean - node_data['arrival_time'].mean

                if available_time > 0 and dist_between_nodes/available_time <= DRONE_MAX_SPEED:
                    G.add_edge(node_name, other_name, custom=True)        

def update_nodes(G, bus_routes, global_time):

    for i in range(len(bus_routes)):
        first_stop = bus_routes[i] # stop currently at or just left
        
        def update_along_route(first_stop):

            # remove nodes that the bus has already passed
            arrival_time = G.nodes[first_stop]['arrival_time'].mean
            prev_stop = first_stop
            while arrival_time < global_time:
                next_stops = [_ for _ in G.neighbors(prev_stop)]
                G.remove_node(prev_stop)
                if len(next_stops) < 1: return
                next_stop = next_stops[0]
                prev_stop = next_stop
                arrival_time = G.nodes[prev_stop]['arrival_time'].mean

            # update list of bus routes to reflect new "starting location"
            bus_routes[i] = prev_stop

            # update estimate for stop we're currently approaching
            next_stop = [_ for _ in G.neighbors(prev_stop)][0]
            new_estimate = update_estimate(G.nodes[next_stop]['arrival_time'], global_time)
            G.nodes[next_stop]['arrival_time'] = new_estimate

            # propogate uncertainty forward along bus route
            # TODO - why is variance needed for anything except closest stop?
            while True:
                prev_stop = next_stop
                next_stops = [_ for _ in G.neighbors(prev_stop)]
                if len(next_stops) < 1: return
                next_stop = next_stops[0]
                travel_time = calc_dist(G.nodes[prev_stop], G.nodes[next_stop]) / BUS_SPEED
                new_mean = G.nodes[prev_stop]['arrival_time'].mean + travel_time
                new_variance = G.nodes[prev_stop]['arrival_time'].variance*BUS_ARRIVAL_VARIANCE*travel_time
                G.nodes[next_stop]['arrival_time'] = RandVar(new_mean, new_variance)

        update_along_route(first_stop)
    # TODO - add stops that have come within planning horizon

    
    
