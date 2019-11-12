import json
import math
import networkx as nx


def add_test1_data(G):
    # drone starting location
    G.add_node('current', x=0, y=0)

    # bus 1 (stop 1)
    G.add_node(1, estimated_arrival_time=1, x=1, y=0)

    # bus 1 (stop 2)
    G.add_node(2, estimated_arrival_time=2, x=3, y=0)

    # bus 1 will always go between stops (riding edge)
    G.add_weighted_edge([(1, 2, 1)])

    # drone goal
    G.add_node('end', x=4, y=0)


def init_grid_from_latlon_bounds(G,
                                 transit_graph,
                                 lat_start=37.675016,
                                 lat_end=37.814703,
                                 lon_start=-122.505734,
                                 lon_end=-122.385306):
    # Default params taken from SF bounding box
    # https://github.com/sisl/MultiAgentAllocationTransit.jl

    # Each cell size is step degrees wide and step degrees tall
    # Rows = Lat = y, Cols = Lon = x
    step = 0.001  # degrees
    # Given step of 0.001 deg,
    # Height and width of each cell is ~ 110 m, hypotenuse is 155 m
    # 140 columns, 120 rows
    num_rows = int(math.ceil(abs(lat_end - lat_start) / float(step)))
    num_cols = int(math.ceil(abs(lon_end - lon_start) / float(step)))

    def get_row(lat): return int(round((lat - lat_start) / float(step)))
    def get_col(lon): return int(round((lon - lon_start) / float(step)))

    # Assume drone starts top left
    G.add_node('current', x=0, y=0)
    # Destination bottom right
    G.add_node('end', x=num_cols - 1, y=num_rows - 1)

    # Each node is named bus_id_stop_id
    # Attributes:
    #   bus_id,
    #   stop_id,
    #   stop_idx - index of stop on this route,
    #   estimated_arrival_time,
    #   x - column within grid,
    #   y - row within grid,
    #   lat - coords of stop,
    #   lon
    for bus_id, bus_stops in enumerate(transit_graph['transit_trips']):
        for i, stop in bus_stops:
            stop_id = stop['stop_id']
            stop_idx = i  # the number of this stop on this bus route

            stop_coords = transit_graph['stop_to_location'][str(stop_id)]
            row = get_row(stop_coords['lat'])  # position in grid
            col = get_col(stop_coords['lon'])

            arrival_time = stop['arrival_time']

            node_id = str(bus_id) + '_' + str(stop_id)
            G.add_node(node_id, bus_id=bus_id, stop_id=stop_id, x=col,
                       y=row, lat=stop_coords['lat'], lon=stop_coords['lon'],
                       estimated_arrival_time=arrival_time)


def add_sf_oct_2019_data(G, transit_graph_file='transit_graph_Oct_2019.json'):
    with open(transit_graph_file, 'r') as f:
        transit_graph = json.load(f)
    print(transit_graph)
    init_grid_from_latlon_bounds(G, transit_graph)


def init_graph(graph_name):
    G = nx.DiGraph()

    if graph_name == 'test1':
        add_test1_data(G)
    elif graph_name == 'sf_oct_2019':
        add_sf_oct_2019_data(G)

    return G
