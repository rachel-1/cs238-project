from utils import *
from local_layer_mdp import *

# TODO - check for cycles?!
# TODO - skip adding edges from unreachable nodes
def add_edges(G, global_time, mdp):
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

            dist = calc_dist(node_data, other_data)
            
            # unconstrained flight
            if other_name == 'end':
                weight = UnconstrainedFlight.get_edge_weight(node_data.get('speed'), dist)
                G.add_edge(node_name, other_name,
                           custom=True, edge_type='unconstrained',
                           weight=weight)
            else: # add constrained edge
                # TODO - account for variance

                if node_name == 'current':
                    available_time = other_data['arrival_time'].mean - global_time
                    if node_data['speed'] != 0 and dist == 0: continue
                else:
                    available_time = other_data['arrival_time'].mean - node_data['arrival_time'].mean

                if available_time > 0 and dist/available_time <= DRONE_MAX_SPEED:
                    speed = node_data['speed']
                    weight = mdp.get_edge_weight(speed, dist, available_time)
                    G.add_edge(node_name, other_name,
                               custom=True, edge_type='constrained',
                               weight=weight)        

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
            print("Estimate for {} just updated to {} from {}".format(next_stop, new_estimate.mean, G.nodes[next_stop]['arrival_time'].mean))
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
                new_variance = G.nodes[prev_stop]['arrival_time'].variance+BUS_ARRIVAL_VARIANCE*travel_time
                G.nodes[next_stop]['arrival_time'] = RandVar(new_mean, new_variance)

        update_along_route(first_stop)
    for node in G.nodes():
        if 'arrival_time' not in G.nodes[node]: continue
        print("node, G.nodes[node]['arrival_time']: ", node, G.nodes[node]['arrival_time'].mean) # TODO - remove debug statement
    # TODO - add stops that have come within planning horizon

    
    
