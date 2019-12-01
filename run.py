import networkx as nx
from graph_setup import init_graph
from visualization import *
from graph_solver import *
from argparse import ArgumentParser
import time


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--skip_viz', action='store_true')
    args = parser.parse_args()
    
    G, bus_routes = init_graph('test1')
    global_time = 0

    if not args.skip_viz:
        display_graph(G, first_time=True)
    
    # global layer
    while True:
        update_nodes(G, bus_routes, global_time)
        add_edges(G, global_time)
        
        if args.debug:
            for edge in G.edges().data():
                print("edge: ", edge)
        
        if not args.skip_viz: display_graph(G)

        mdps = calc_edge_weights(G, global_time)

        if args.debug:
            for edge in G.edges().data():
                print("edge: ", edge)
                
        path = nx.astar_path(G, 'current', 'end')
        
        if args.debug:
            print(path)

        # local layer
        num_local_steps_taken = 0
        for idx in range(len(path)-1):
            mdp = mdps[(path[idx], path[idx+1])]
            next_node = G.nodes[path[idx+1]]
            if hasattr(mdp, 'horizon'):
                print("mdp.horizon: ", mdp.horizon) # TODO - remove debug statement
                print("global_time: ", global_time) # TODO - remove debug statement
                remaining_time = next_node['arrival_time'].mean - global_time
                if mdp.horizon != remaining_time:
                    print("mdp.horizon: ", mdp.horizon) # TODO - remove debug statement
                    print("remaining_time: ", remaining_time) # TODO - remove debug statement
                    print("Delay/speed-up")
                    break # there was a delay or speed-up, so replan
            # TODO - break off in the middle of execution if needed
            was_successful, steps_taken = run_policy(mdp, G, path[idx+1], global_time, not args.skip_viz)
            num_local_steps_taken += steps_taken
            
            if not was_successful:
                print("Failed!")
                break # allow global layer to replan
            if num_local_steps_taken > MAX_LOCAL_STEPS: break

        # if we made it to the final goal
        if G.nodes['current']['speed'] == 0 \
           and G.nodes['current']['x'] == G.nodes['end']['x'] \
           and G.nodes['current']['y'] == G.nodes['end']['y']:
            break

        # clear "custom" edges (i.e. not connections between buses)
        edges_to_remove = [edge for edge in G.edges.data() if edge[2]['custom']]
        G.remove_edges_from(edges_to_remove)

