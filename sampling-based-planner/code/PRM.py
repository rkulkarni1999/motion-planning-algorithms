# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy import spatial
import math
import random
    
# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size

        self.samples = []  # list of sampled points
        self.graph = nx.Graph()  # constructed graph
        self.path = []  # list of nodes of the found path

        # PRM Parameters. 
        self.step_count_collision = 100
        self.uniform_step_size = 10
        self.gaussian_covariance_matrix = np.array([[350, 0], [0, 350]])
        self.bridge_covariance_matrix = [[650, 0],[0, 650]]
        self.sampling_radius = 20
        self.k_neighbours = 50

    def check_collision(self, p1, p2):
        """Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        """
        d_row, d_col = p2[0] - p1[0], p2[1] - p1[1]
        
        row_step = d_row / self.step_count_collision
        col_step = d_col / self.step_count_collision

        for step in range(self.step_count_collision):
            
            row_point = p1[0] + step * row_step
            col_point = p1[1] + step * col_step

            if self.map_array[int(row_point)][int(col_point)] == 0:
                return True

        return False

    def dis(self, point1, point2):
        """Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        """
        eucledian_distance = math.sqrt(math.pow((point1[0] - point2[0]), 2) + math.pow((point1[1] - point2[1]), 2))
        return eucledian_distance
    
    
    def uniform_sample(self, n_pts):
        """Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        """
        # Initialize graph
        self.graph.clear()
        
        ### YOUR CODE HERE ###
        for _ in range(n_pts):
            
            random_row_index = np.random.randint(0, self.size_row)
            random_col_index = np.random.randint(0, self.size_col)
            random_point = [random_row_index, random_col_index]

            if(self.map_array[random_point[0]][random_point[1]] == 1):
                self.samples.append((random_point))

    def gaussian_sample(self, n_pts):
        """Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        """
        # Initialize graph
        self.graph.clear()
        
        for _ in range(n_pts):
            
            # Generating a random point. 
            random_row_index = np.random.randint(0, self.size_row)
            random_col_index = np.random.randint(0, self.size_col)
            random_point_1 = [random_row_index, random_col_index]
            
            # Generate a new point using a Gaussian distribution centered around random point
            gaussian_point_2 = np.random.multivariate_normal(random_point_1, self.gaussian_covariance_matrix, size=1)[0]
            gaussian_point_2 = [int(coord) for coord in gaussian_point_2]
            
            # Extract row and column coordinates of gaussian_point_2
            row_coord, col_coord = gaussian_point_2

            # Check if the row coordinate is out of bounds
            if row_coord > self.size_row - 1:
                continue

            # Check if the column coordinate is out of bounds
            if col_coord > self.size_col - 1:
                continue

            # Extract values from the map_array for random_point_1 and gaussian_point_2
            value_at_random_point_1 = self.map_array[random_point_1[0]][random_point_1[1]]
            value_at_gaussian_point_2 = self.map_array[gaussian_point_2[0]][gaussian_point_2[1]]

            # Check if the values at these points are the same
            if value_at_random_point_1 == value_at_gaussian_point_2:
                continue  # Skip this point if they are equal

            # Determine the point to append to self.samples based on the value at random_point_1
            point_to_append = tuple(random_point_1) if self.map_array[random_point_1[0]][random_point_1[1]] == 1 else tuple(gaussian_point_2)

            # Append the determined point to self.samples
            self.samples.append(point_to_append)
        
    def bridge_sample(self, n_pts):
        """Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        """
        # Initialize graph
        self.graph.clear()

        for _ in range(n_pts):

            # Generate random row and column indices within the map boundaries
            random_row_index = np.random.randint(0, self.size_row)
            random_col_index = np.random.randint(0, self.size_col)
            random_point_1 = [random_row_index, random_col_index]
            
            # Extract the value at random_point_1 from the map_array
            map_value_at_random_point_1 = self.map_array[random_point_1[0]][random_point_1[1]]

            # Check if the value at random_point_1 is equal to 1 (indicating an obstacle)
            if map_value_at_random_point_1 == 1:
                continue  # Skip this point if it's an obstacle

            # Generate a new point bridge_point_2 using the bridge covariance matrix centered around random_point_1
            bridge_point_2 = np.random.multivariate_normal(random_point_1, self.bridge_covariance_matrix, size=1)[0]
            bridge_point_2 = [int(coord) for coord in bridge_point_2]
            
            # Extract row and column coordinates of bridge_point_2
            bridge_point_2_row, bridge_point_2_col = bridge_point_2

            # Check if bridge_point_2 is out of bounds
            if (bridge_point_2_row > self.size_row - 1) or (bridge_point_2_col > self.size_col - 1):
                continue  # Skip this point if it's out of bounds

            # Check if bridge_point_2 is on an obstacle (value 0 in map_array)
            if self.map_array[bridge_point_2_row][bridge_point_2_col] == 0:
                # Calculate the midpoint between random_point_1 and bridge_point_2
                mid = [(a + b) // 2 for a, b in zip(random_point_1, bridge_point_2)]

                # Check if the midpoint is on an obstacle (value 1 in map_array)
                if self.map_array[mid[0]][mid[1]] == 1:
                    self.samples.append(tuple(mid))

    def draw_map(self):
        """Visualization of the result"""
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict(zip(range(len(self.samples)), node_pos))
        pos["start"] = (self.samples[-2][1], self.samples[-2][0])
        pos["goal"] = (self.samples[-1][1], self.samples[-1][0])

        # draw constructed graph
        nx.draw(
            self.graph, pos, node_size=3, node_color="y", edge_color="y", ax=ax
        )

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(
                self.graph,
                pos=pos,
                nodelist=self.path,
                node_size=8,
                node_color="b",
            )
            nx.draw_networkx_edges(
                self.graph,
                pos=pos,
                edgelist=final_path_edge,
                width=2,
                edge_color="b",
            )

        # draw start and goal
        nx.draw_networkx_nodes(
            self.graph,
            pos=pos,
            nodelist=["start"],
            node_size=12,
            node_color="g",
        )
        nx.draw_networkx_nodes(
            self.graph, pos=pos, nodelist=["goal"], node_size=12, node_color="r"
        )

        # show image
        plt.axis("on")
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()

    def sample(self, n_pts=1000, sampling_method="uniform"):
        """Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        """
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        # ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01),
        #          (p_id0, p_id2, weight_02),
        #          (p_id1, p_id2, weight_12) ...]
        pairs = []
        
        kdtrees = spatial.KDTree(self.samples)
        p_ids = kdtrees.query_pairs(self.sampling_radius)
        
        for pid in p_ids:
            p1 = pid[0]
            p2 = pid[1]
            wt = self.dis(self.samples[p1], self.samples[p2])
            if self.check_collision(list(self.samples[p1]), list(self.samples[p2])):
                continue
            pairs.append((p1, p2, wt))

        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01),
        #                                     (p_id0, p_id2, weight_02),
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        self.graph.add_nodes_from([])
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print(
            "The constructed graph has %d nodes and %d edges"
            % (n_nodes, n_edges)
        )

    def search(self, start, goal):
        """Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal nodes, edges of them
        and their nearest neighbors to graph for
        self.graph to search for a path.
        """
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(["start", "goal"])

        # ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0),
        #                (start_id, p_id1, weight_s1),
        #                (start_id, p_id2, weight_s2) ...]
        start_pairs = []
        goal_pairs = []

        # Create a KDTree from the self.samples data
        kdtree = spatial.KDTree(self.samples)

        # Query the KDTree to find the k_neighbours nearest points to start and goal
        distances, neighbor_indices = kdtree.query([start, goal], k=self.k_neighbours)

        # Initialize lists to store the start and goal pairs
        start_pairs = []
        goal_pairs = []

        # Iterate through the neighbor indices to create start and goal pairs
        for ii in range(self.k_neighbours):
            # Extract neighbor indices for start and goal points
            neighbor_index_start = neighbor_indices[0][ii]
            neighbor_index_goal = neighbor_indices[1][ii]

            # Calculate the distance between start and the nearest neighbor
            distance_start = self.dis(self.samples[neighbor_index_start], start)
            start_pairs.append(('start', neighbor_index_start, distance_start))

            # Calculate the distance between goal and the nearest neighbor
            distance_goal = self.dis(self.samples[neighbor_index_goal], goal)
            goal_pairs.append(('goal', neighbor_index_goal, distance_goal))

        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)

        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(
                self.graph, "start", "goal"
            )
            path_length = (
                nx.algorithms.shortest_paths.weighted.dijkstra_path_length(
                    self.graph, "start", "goal"
                )
            )
            print("The path length is %.2f" % path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")

        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(["start", "goal"])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
