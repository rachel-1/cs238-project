import networkx as nx
import json


def add_test1_data(G):
    # drone starting location
    G.add_node('current', x=0, y=0)

    # bus 1 (stop 1)
    G.add_node(1, estimated_arrival_time=1, x=1, y=0)

    # bus 1 (stop 2)
    G.add_node(2, estimated_arrival_time=2, x=3, y=0)

    # bus 1 will always go between stops (riding edge)
    G.add_edge(1, 2)

    # drone goal
    G.add_node('end', x=4, y=0)


def init_grid_from_latlon_bounds(transit_graph,
                                 lat_start=37.675016,
                                 lat_end=37.814703,
                                 lon_start=-122.505734,
                                 lon_end=-122.385306):
    # Default params taken from SF bounding box
    # https://github.com/sisl/MultiAgentAllocationTransit.jl
    # Rows = Lat = y, Cols = Long = x
    # Learn how large cells can be using average differences

    return


def add_sf_oct_2019_data(G, transit_graph_file='transit_graph_Oct_2019.json'):
    with open(transit_graph_file, 'r') as f:
        data = json.load(f)

    return


def init_graph(graph_name):
    G = nx.DiGraph()

    if graph_name == 'test1':
        add_test1_data(G)

    return G
