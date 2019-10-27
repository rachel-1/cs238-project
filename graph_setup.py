import networkx as nx

def add_test1_data(G):
    # drone starting location
    G.add_node('current', x=0, y=0)

    # bus 1 (stop 1)
    G.add_node(1, estimated_arrival_time=1, x=1, y=0)

    # bus 1 (stop 2)
    G.add_node(2, estimate_arrival_time=2, x=3, y=0)

    # bus 1 will always go between stops (riding edge)
    G.add_edge(1,2)

    # drone goal
    G.add_node('end', x=4, y=0)

def init_graph(graph_name):
    G = nx.DiGraph()
    
    if graph_name == 'test1':
        add_test1_data(G)

    return G
