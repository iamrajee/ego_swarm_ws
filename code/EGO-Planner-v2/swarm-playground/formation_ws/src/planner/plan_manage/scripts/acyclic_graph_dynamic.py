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
    # (2, 4, 1),
    # (2, 4, 1),
    # (2, 4, 1),
    # Add more coordinates as needed
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
    # plt.show(block=False)
    # plt.pause(0.2)
    # plt.close()
    # plt.show()
    # plt.close()

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
# for i in range(0,10): 
def update(i): 
    ax.clear()
    make_acyclic(G)

    # Extract x, y, z coordinates for plotting
    pos = nx.get_node_attributes(G, 'pos')
    x = [pos[node][0] for node in G.nodes()]
    y = [pos[node][1] for node in G.nodes()]
    z = [pos[node][2] for node in G.nodes()]

    # Draw the graph

    # fig = plt.figure(figsize=(8, 6))
    # ax = fig.add_subplot(111, projection='3d')

    plot_graph(fig,ax,i,x,y,z,pos)
    print("count:",i)
    # plt.show()
    # ax.clear()
ani = FuncAnimation(fig, update, frames=10, interval=1000, repeat=False)
plt.show()
# plt.show()

'''
#working animation
import networkx as nx
import matplotlib.pyplot as plt
from itertools import combinations
from matplotlib.animation import FuncAnimation
import random

# Function to calculate Euclidean distance between two points
def distance(point1, point2):
    return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2) ** 0.5

# Initial set of XYZ coordinates
initial_coordinates = [
    (0, 0, 0),
    (1, 2, 3),
    (2, 4, 1),
    # Add more coordinates as needed
]

# Create a figure and axis for the plot
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# Create an empty graph
G = nx.Graph()

# Number of iterations
num_iterations = 10

# Generate an empty plot for initialization
nodes = nx.draw_networkx_nodes(G, pos={}, ax=ax, node_size=500, node_color='skyblue', alpha=0.8)
edges = nx.draw_networkx_edges(G, pos={}, ax=ax, edge_color='black', alpha=0.5)

# Update function for animation
def update(iteration):
    ax.clear()

    # Generate slightly altered coordinates for this iteration
    coordinates = [tuple(coord + random.uniform(-0.5, 0.5) for coord in point) for point in initial_coordinates]

    # Clear existing graph and add nodes to the graph
    G.clear()
    for i, coord in enumerate(coordinates):
        G.add_node(i, pos=coord)

    # Calculate distances and add edges
    for pair in combinations(G.nodes, 2):
        node1, node2 = pair
        dist = distance(G.nodes[node1]['pos'], G.nodes[node2]['pos'])
        G.add_edge(node1, node2, weight=dist)

    # Check for cycles and break them if they exist
    while nx.cycle_basis(G):
        cycle = nx.cycle_basis(G)[0]  # Select a cycle
        cycle_edges = list(zip(cycle, cycle[1:] + [cycle[0]]))  # Get the edges of the cycle
        edge_to_remove = random.choice(cycle_edges)  # Choose a random edge from the cycle
        G.remove_edge(*edge_to_remove)  # Remove the selected edge from the cycle

    # Extract x, y, z coordinates for plotting
    pos = nx.get_node_attributes(G, 'pos')
    x = [pos[node][0] for node in G.nodes()]
    y = [pos[node][1] for node in G.nodes()]
    z = [pos[node][2] for node in G.nodes()]

    # Plot nodes and edges
    ax.scatter(x, y, z, c='skyblue', s=500, edgecolors='black', depthshade=True)
    for edge in G.edges():
        start = pos[edge[0]]
        end = pos[edge[1]]
        edge_x = [start[0], end[0]]
        edge_y = [start[1], end[1]]
        edge_z = [start[2], end[2]]
        ax.plot(edge_x, edge_y, edge_z, c='black')

    ax.set_title(f"Iteration {iteration + 1}: Acyclic Graph from XYZ Coordinates")
    ax.set_axis_off()

# Create animation
ani = FuncAnimation(fig, update, frames=num_iterations, interval=1000, repeat=False)
plt.show()

'''

'''
# close replica
import networkx as nx
import matplotlib.pyplot as plt
from itertools import combinations
import random

# Function to calculate Euclidean distance between two points
def distance(point1, point2):
    return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2) ** 0.5

# Initial set of XYZ coordinates
initial_coordinates = [
    (0, 0, 0),
    (1, 2, 3),
    (2, 4, 1),
    # Add more coordinates as needed
]

# Number of iterations
num_iterations = 10

for iteration in range(num_iterations):
    print(f"Iteration: {iteration + 1}")
    
    # Create an empty graph
    G = nx.Graph()

    # Generate slightly altered coordinates for this iteration
    coordinates = [tuple(coord + random.uniform(-0.5, 0.5) for coord in point) for point in initial_coordinates]

    # Add nodes to the graph
    for i, coord in enumerate(coordinates):
        G.add_node(i, pos=coord)

    # Calculate distances and add edges
    for pair in combinations(G.nodes, 2):
        node1, node2 = pair
        dist = distance(G.nodes[node1]['pos'], G.nodes[node2]['pos'])
        G.add_edge(node1, node2, weight=dist)

    # Check for cycles and break them if they exist
    while nx.cycle_basis(G):
        cycle = nx.cycle_basis(G)[0]  # Select a cycle
        cycle_edges = list(zip(cycle, cycle[1:] + [cycle[0]]))  # Get the edges of the cycle
        edge_to_remove = random.choice(cycle_edges)  # Choose a random edge from the cycle
        G.remove_edge(*edge_to_remove)  # Remove the selected edge from the cycle

    # Extract x, y, z coordinates for plotting
    pos = nx.get_node_attributes(G, 'pos')
    x = [pos[node][0] for node in G.nodes()]
    y = [pos[node][1] for node in G.nodes()]
    z = [pos[node][2] for node in G.nodes()]

    # Draw the graph
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(x, y, z, c='skyblue', s=500, edgecolors='black', depthshade=True)

    for edge in G.edges():
        start = pos[edge[0]]
        end = pos[edge[1]]
        edge_x = [start[0], end[0]]
        edge_y = [start[1], end[1]]
        edge_z = [start[2], end[2]]
        ax.plot(edge_x, edge_y, edge_z, c='black')

    plt.title(f"Iteration {iteration + 1}: Acyclic Graph from XYZ Coordinates")
    plt.axis('off')
    plt.show()


'''


'''
#orignal
import networkx as nx
import matplotlib.pyplot as plt
from itertools import combinations

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
    # (2, 4, 1),
    # (2, 4, 1),
    # (2, 4, 1),
    # Add more coordinates as needed
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

def plot_graph(fig, ax):
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

    plt.title("Acyclic Graph from XYZ Coordinates")
    plt.axis('off')
    plt.show()
    plt.close()

for i in range(0,10):  
    make_acyclic(G)

    # Extract x, y, z coordinates for plotting
    pos = nx.get_node_attributes(G, 'pos')
    x = [pos[node][0] for node in G.nodes()]
    y = [pos[node][1] for node in G.nodes()]
    z = [pos[node][2] for node in G.nodes()]

    # Draw the graph
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    plot_graph(fig,ax)
    print("count:",i)
    
'''