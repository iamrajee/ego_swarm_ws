import networkx as nx
import matplotlib.pyplot as plt
from itertools import combinations
from matplotlib.animation import FuncAnimation

# Function to calculate Euclidean distance between two points
def distance(point1, point2):
    return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2) ** 0.5

# Given set of XYZ coordinates
coordinates = [
    (0, 0, 0),
    (1, 2, 3),
    (2, 1, 0),
    (2, 4, 1),
    (4, 2, 1),
    (2, 1, 1),
    (3, 5, 6),
    (3, 5, 7),
    (3, 6, 0),
    (4, 6, 0),
    (5, 6, 0),
    (6, 6, 0),
]

# Create an empty graph
G = nx.Graph()

# Add nodes to the graph
for i, coord in enumerate(coordinates):
    G.add_node(i, pos=coord)

# Calculate distances and add edges
for pair in combinations(G.nodes, 2):
    node1, node2 = pair
    dist = distance(G.nodes[node1]['pos'], G.nodes[node2]['pos'])
    G.add_edge(node1, node2, weight=dist)

# Remove edges to make the graph acyclic
def make_acyclic(graph):
    for edge in graph.edges():
        graph.remove_edge(*edge)
        if nx.is_connected(graph):
            continue
        else:
            graph.add_edge(*edge)  # Re-add the edge if it breaks connectivity

def plot_graph(fig, ax,i,x,y,z,pos):
    # Plot nodes
    ax.scatter(x, y, z, c='skyblue', s=500, edgecolors='black', depthshade=True)

    # Plot edges
    for edge in G.edges():
        start = pos[edge[0]]
        end = pos[edge[1]]
        edge_x = [start[0], end[0]]
        edge_y = [start[1], end[1]]
        edge_z = [start[2], end[2]]
        ax.plot(edge_x, edge_y, edge_z, c='black')

    plt.title(str(i)+":Acyclic Graph from XYZ Coordinates")
    plt.axis('off')

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
def update(i): 
    ax.clear()
    make_acyclic(G)
    # Extract x, y, z coordinates for plotting
    pos = nx.get_node_attributes(G, 'pos')
    x = [pos[node][0] for node in G.nodes()]
    y = [pos[node][1] for node in G.nodes()]
    z = [pos[node][2] for node in G.nodes()]
    plot_graph(fig,ax,i,x,y,z,pos)
    print("count:",i)
ani = FuncAnimation(fig, update, frames=10, interval=1000, repeat=False)
plt.show()