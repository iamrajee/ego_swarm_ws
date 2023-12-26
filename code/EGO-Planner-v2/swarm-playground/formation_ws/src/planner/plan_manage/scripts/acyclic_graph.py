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

make_acyclic(G)

# Extract x, y, z coordinates for plotting
pos = nx.get_node_attributes(G, 'pos')
x = [pos[node][0] for node in G.nodes()]
y = [pos[node][1] for node in G.nodes()]
z = [pos[node][2] for node in G.nodes()]

# Draw the graph
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

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

'''
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
    (2, 4, 1),
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

make_acyclic(G)

# Extract x, y, z coordinates for plotting
pos = nx.get_node_attributes(G, 'pos')
x = [pos[node][0] for node in G.nodes()]
y = [pos[node][1] for node in G.nodes()]
z = [pos[node][2] for node in G.nodes()]

# Draw the graph
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

nx.draw(G, pos, ax=ax, node_size=500, node_color='skyblue', font_weight='bold')

# Scatter plot for node positions
ax.scatter(x, y, z, c='skyblue', s=500, edgecolors='black', depthshade=True)

plt.title("Acyclic Graph from XYZ Coordinates")
plt.axis('off')
plt.show()
'''

'''
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
    (2, 4, 1),
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

make_acyclic(G)

# Draw the graph
pos = nx.get_node_attributes(G, 'pos')

plt.figure(figsize=(8, 6))
nx.draw(G, pos, with_labels=True, node_size=500, node_color='skyblue', font_weight='bold')
plt.title("Acyclic Graph from XYZ Coordinates")
plt.axis('off')

# Show edge labels if they exist
if all('weight' in G.edges[edge] for edge in G.edges()):
    edge_labels = {(u, v): G.edges[u, v]['weight'] for u, v in G.edges()}
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

plt.show()
'''

'''
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
    (2, 4, 1),
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

make_acyclic(G)

# Draw the graph
pos = nx.get_node_attributes(G, 'pos')
edge_labels = {(u, v): d['weight'] for u, v, d in G.edges(data=True)}

plt.figure(figsize=(8, 6))
nx.draw(G, pos, with_labels=True, node_size=500, node_color='skyblue', font_weight='bold')
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
plt.title("Acyclic Graph from XYZ Coordinates")
plt.axis('off')
plt.show()
'''

'''
import networkx as nx
import matplotlib.pyplot as plt
from itertools import combinations

# Function to calculate Euclidean distance between two points
def distance(point1, point2):
    return ((point1[0] - point2[0])*2 + (point1[1] - point2[1])*2 + (point1[2] - point2[2])*2) * 0.5

# Given set of XYZ coordinates
coordinates = [
    (0, 0, 0),
    (1, 2, 3),
    (2, 4, 1),
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
cycles = list(nx.simple_cycles(G))
if cycles:
    # If cycles exist, remove edges causing cycles (e.g., using a heuristic like removing longest edge)
    for cycle in cycles:
        longest_edge = max(G.edges(cycle), key=lambda x: G.edges[x]['weight'])
        G.remove_edge(*longest_edge)

# Draw the graph
pos = nx.get_node_attributes(G, 'pos')
edge_labels = {(u, v): d['weight'] for u, v, d in G.edges(data=True)}

plt.figure(figsize=(8, 6))
nx.draw(G, pos, with_labels=True, node_size=500, node_color='skyblue', font_weight='bold')
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
plt.title("Acyclic Graph from XYZ Coordinates")
plt.axis('off')
plt.show()
'''