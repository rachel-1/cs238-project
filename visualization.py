import matplotlib.pyplot as plt
import networkx as nx

def visualize_probabilities(T):
    plt.imshow(T, cmap='hot', interpolation='nearest')
    plt.show()
    
def display_graph(G):
    pos = nx.random_layout(G)
    nx.draw_networkx_nodes(G,pos)
    nx.draw_networkx_edges(G,pos,arrowstyle='->')
    nx.draw_networkx_labels(G,pos)

    plt.show()
    #plt.savefig('graph.png', dpi=200)
