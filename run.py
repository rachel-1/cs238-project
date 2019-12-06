import networkx as nx
import numpy as np
from graph_setup import init_graph
from visualization import *
from graph_solver import *
from argparse import ArgumentParser
import time


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--skip_viz', action='store_true')
    parser.add_argument('--beta', type=float, default=1.0,
                        help="""Beta value between 0 and 1, where closer to 1
                        means greater the risk we take for connections. 1 means
                        never abort (but may still be interrupted by global
                        layer replanning.""")
    args = parser.parse_args()

    G, bus_routes = init_graph('sf_oct_2019')

    mdp = ConstrainedFlight(max_distance=1, max_timesteps=1)
    mdp.solve()

    global_time = 0
    total_accel = 0

    if not args.skip_viz:
        display_graph(G, first_time=True)

    replan = False
    heuristic = lambda n0, n1: calc_dist(G.nodes[n0], G.nodes[n1]) # for A*

    # global layer
    while True:
        update_nodes(G, bus_routes, global_time)
        add_edges(G, global_time, mdp)

        if args.debug:
            for edge in G.edges().data():
                print("edge: ", edge)

        if not args.skip_viz: display_graph(G)

        worst_value = min(mdp.value_func)
        min_value = worst_value * args.beta # lower bound for value func

        if args.debug:
            for edge in G.edges().data():
                print("edge: ", edge)

        # first iteration
        if global_time == 0:
            path = nx.astar_path(G, 'current', 'end', heuristic=heuristic)
            # get set of candidate paths
            candidates = get_candidate_paths(path, mdp, G, global_time, min_value)

        # Open layer replanning or aborting constrained flight
        if replan:
            path = candidates.pop(0)
            # recalculate candidates too?
            candidates = get_candidate_paths(path, G, global_time, min_value)
            replan = False

        if args.debug:
            print("Best path: ", path)
            print("Candidate paths: ", candidates)

        # local layer
        num_local_steps_taken = 0
        for idx in range(len(path)-1):
            next_node = G.nodes[path[idx+1]]
            edge_type = G.get_edge_data(path[idx], path[idx+1])['edge_type']

            if edge_type == 'unconstrained':
                policy = UnconstrainedFlight()
            else:
                remaining_time = next_node['arrival_time'].mean - global_time
                if edge_type == 'riding':
                    policy = Riding()
                elif edge_type == 'constrained':
                    dist = calc_dist(G.nodes[path[idx]], next_node)
                    policy = mdp

                # TODO check if there was a delay or speed-up, so replan

            # TODO - break off in the middle of execution if needed
            aborted, was_successful, steps_taken, accel_amt = run_policy(policy, G, path[idx+1], global_time, min_value, not args.skip_viz)
            if aborted:
                print("Aborted path, replanning!")
                break # allow global layer to replan

            global_time += steps_taken
            num_local_steps_taken += steps_taken
            total_accel += accel_amt

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


    total_time_taken = global_time
    ENERGY_PER_ACCEL_UNIT = 1
    total_energy_used = ENERGY_PER_ACCEL_UNIT * total_accel
    print("Total time taken: ", total_time_taken)
    print("Total energy used: ", total_energy_used)
