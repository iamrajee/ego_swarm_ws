import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

import networkx as nx
import matplotlib.pyplot as plt
from itertools import combinations
from matplotlib.animation import FuncAnimation

# Function to calculate Euclidean distance between two points
def distance(point1, point2):
    return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2) ** 0.5

# Remove edges to make the graph acyclic
def make_acyclic(graph):
    for edge in graph.edges():
        graph.remove_edge(*edge)
        if nx.is_connected(graph):
            continue
        else:
            graph.add_edge(*edge)  # Re-add the edge if it breaks connectivity
            
# def plot_graph(fig, ax,i,x,y,z,pos,G):
#     # Plot nodes
#     ax.scatter(x, y, z, c='skyblue', s=500, edgecolors='black', depthshade=True)

#     # Plot edges
#     for edge in G.edges():
#         start = pos[edge[0]]
#         end = pos[edge[1]]
#         edge_x = [start[0], end[0]]
#         edge_y = [start[1], end[1]]
#         edge_z = [start[2], end[2]]
#         ax.plot(edge_x, edge_y, edge_z, c='black')

#     plt.title(str(i)+":Acyclic Graph from XYZ Coordinates")
#     plt.axis('off')
    
#     # plt.show(block=False)
#     # plt.pause(1)
#     # plt.close()

def update(i): 
    global G
    if G.edges() == []:
        return
    global fig
    global ax
    ax.clear()
    make_acyclic(G)
    # Extract x, y, z coordinates for plotting
    pos = nx.get_node_attributes(G, 'pos')
    x = [pos[node][0] for node in G.nodes()]
    y = [pos[node][1] for node in G.nodes()]
    z = [pos[node][2] for node in G.nodes()]
    # plot_graph(fig,ax,i,x,y,z,pos,G)
    # print("count:",i)
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

G = nx.Graph()
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
        
class PlanningVisualization:
    def __init__(self):
        # self.node = nh
        # self.goal_point_pub = rospy.Publisher("goal_point", Marker, queue_size=2)
        # self.global_list_pub = rospy.Publisher("global_list", Marker, queue_size=2)
        # self.init_list_pub = rospy.Publisher("init_list", Marker, queue_size=2)
        # ... other publishers ...

        self.swarm_formation_visual_pub = rospy.Publisher("swarm_graph_visual_py", MarkerArray, queue_size=10)

        self.t_init = rospy.Time.now()
        self.formation_size_ = rospy.get_param("/drone_0_ego_planner_node/formation/num", 8)
        self.drone_id_ = rospy.get_param("manager/drone_id", 0)
        self.formation_type_ = rospy.get_param("optimization/formation_type", 1)
        # self.init_swarm_graph_visual()
        self.swarm_odom = [np.zeros(3) for _ in range(self.formation_size_)]

        self.drone_0_odom_sub_ = rospy.Subscriber("/drone_0_visual_slam/odom", Odometry, self.drone_0_odom_callback)
        self.drone_1_odom_sub_ = rospy.Subscriber("/drone_1_visual_slam/odom", Odometry, self.drone_1_odom_callback)
        self.drone_2_odom_sub_ = rospy.Subscriber("/drone_2_visual_slam/odom", Odometry, self.drone_2_odom_callback)
        self.drone_3_odom_sub_ = rospy.Subscriber("/drone_3_visual_slam/odom", Odometry, self.drone_3_odom_callback)
        self.drone_4_odom_sub_ = rospy.Subscriber("/drone_4_visual_slam/odom", Odometry, self.drone_4_odom_callback)
        self.drone_5_odom_sub_ = rospy.Subscriber("/drone_5_visual_slam/odom", Odometry, self.drone_5_odom_callback)
        self.drone_6_odom_sub_ = rospy.Subscriber("/drone_6_visual_slam/odom", Odometry, self.drone_6_odom_callback)
        self.drone_7_odom_sub_ = rospy.Subscriber("/drone_7_visual_slam/odom", Odometry, self.drone_7_odom_callback)
        self.drone_8_odom_sub_ = rospy.Subscriber("/drone_8_visual_slam/odom", Odometry, self.drone_8_odom_callback)
        self.drone_9_odom_sub_ = rospy.Subscriber("/drone_9_visual_slam/odom", Odometry, self.drone_9_odom_callback)
        # ... other subscribers ...

        print("formation_size_: ", self.formation_size_)
        
        if self.drone_id_ == 0:
            rospy.Timer(rospy.Duration(0.01), self.swarm_graph_visul_callback)
            # fig = plt.figure(figsize=(8, 6))
            # ax = fig.add_subplot(111, projection='3d')
        
            ani = FuncAnimation(fig, update, frames=100000, interval=100, repeat=False)
            plt.show()


    def swarm_graph_visul_callback(self, e):
        self.acyclic_swarm_visual()
        if self.line_size_ == 0:
            return
        # self.acyclic_swarm_visual()
        lines = MarkerArray()
        for i in range(self.line_size_):
            line_strip = Marker()
            line_strip.header.frame_id = "world"
            line_strip.header.stamp = rospy.Time.now()
            line_strip.type = Marker.LINE_STRIP
            line_strip.action = Marker.ADD
            line_strip.id = i

            line_strip.scale.x = 0.1
            line_strip.color.r = 0.3
            line_strip.color.g = 0.3
            line_strip.color.b = 0.9
            line_strip.color.a = 0.8

            p = Point()
            q = Point()
            p.x = self.swarm_odom[self.line_begin_[i]][0]
            p.y = self.swarm_odom[self.line_begin_[i]][1]
            p.z = self.swarm_odom[self.line_begin_[i]][2]

            q.x = self.swarm_odom[self.line_end_[i]][0]
            q.y = self.swarm_odom[self.line_end_[i]][1]
            q.z = self.swarm_odom[self.line_end_[i]][2]
            line_strip.points.append(p)
            line_strip.points.append(q)

            lines.markers.append(line_strip)
        self.swarm_formation_visual_pub.publish(lines)

    def init_swarm_graph_visual(self):
        if self.formation_type_ == 0:
            self.formation_size_ = 0
            self.line_size_ = 0
        elif self.formation_type_ == 1:
            self.formation_size_ = 4
            self.line_size_ = 6
            self.line_begin_ = [0, 1, 2, 3, 0, 1]
            self.line_end_ = [1, 2, 3, 0, 2, 3]
            # Optionally, you might want to resize these lists
            # self.line_begin_.resize(self.line_size_)
            # self.line_end_.resize(self.line_size_)
        else:
            pass  # Handle other formation types if needed

    def acyclic_swarm_visual(self):
        global G
      
        if self.formation_type_ == 0:
            self.formation_size_ = 0
            self.line_size_ = 0
            return
        # elif self.formation_type_ == 1:
        coordinates=self.swarm_odom
        G = nx.Graph()

        # Add nodes to the graph
        for i, coord in enumerate(coordinates):
            G.add_node(i, pos=coord)

        # Calculate distances and add edges
        for pair in combinations(G.nodes, 2):
            node1, node2 = pair
            dist = distance(G.nodes[node1]['pos'], G.nodes[node2]['pos'])
            G.add_edge(node1, node2, weight=dist)
        make_acyclic(G)
        
        self.line_size_= len(G.edges())
        self.line_begin_=[]
        self.line_end_ = []
        for edge in G.edges():
            # print(edge[0], edge[1] )
            self.line_begin_.append(edge[0])
            self.line_end_.append(edge[1])
        
        # fig = plt.figure(figsize=(8, 6))
        # ax = fig.add_subplot(111, projection='3d')
        # def update(i): 
        #     ax.clear()
        #     make_acyclic(G)
        #     # Extract x, y, z coordinates for plotting
        #     pos = nx.get_node_attributes(G, 'pos')
        #     x = [pos[node][0] for node in G.nodes()]
        #     y = [pos[node][1] for node in G.nodes()]
        #     z = [pos[node][2] for node in G.nodes()]
        #     plot_graph(fig,ax,i,x,y,z,pos)
        #     print("count:",i)
        # ani = FuncAnimation(fig, update, frames=10, interval=1000, repeat=False)
        # plt.show()
        
        #============
        # fig = plt.figure(figsize=(8, 6))
        # ax = fig.add_subplot(111, projection='3d')
        # pos = nx.get_node_attributes(G, 'pos')
        # x = [pos[node][0] for node in G.nodes()]
        # y = [pos[node][1] for node in G.nodes()]
        # z = [pos[node][2] for node in G.nodes()]
        # plot_graph(fig,ax," ",x,y,z,pos,G)
            

    # Define other methods...
    def drone_0_odom_callback(self, msg):
        if self.formation_size_ <= 0:
            return
        self.swarm_odom[0] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
    
    def drone_1_odom_callback(self, msg):
        if self.formation_size_ <= 0:
            return
        self.swarm_odom[1] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        
    def drone_2_odom_callback(self, msg):
        if self.formation_size_ <= 0:
            return
        self.swarm_odom[2] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        
    def drone_3_odom_callback(self, msg):
        if self.formation_size_ <= 0:
            return
        self.swarm_odom[3] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

    def drone_4_odom_callback(self, msg):
        if self.formation_size_ <= 0:
            return
        self.swarm_odom[4] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        
    def drone_5_odom_callback(self, msg):
        if self.formation_size_ <= 0:
            return
        self.swarm_odom[5] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        
    def drone_6_odom_callback(self, msg):
        if self.formation_size_ <= 0:
            return
        self.swarm_odom[6] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        
    def drone_7_odom_callback(self, msg):
        if self.formation_size_ <= 0:
            return
        self.swarm_odom[7] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        
    def drone_8_odom_callback(self, msg):
        if self.formation_size_ <= 0:
            return
        self.swarm_odom[8] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        
    def drone_9_odom_callback(self, msg):
        if self.formation_size_ <= 0:
            return
        self.swarm_odom[9] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        
    
if __name__ == "__main__":
    rospy.init_node("planning_visualization_node")
    planner = PlanningVisualization()#rospy.get_node("~"))
    rospy.spin()
