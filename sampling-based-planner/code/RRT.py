# Standard Algorithm Implementation
# Sampling-based Algorithms RRT

import matplotlib.pyplot as plt
import numpy as np
import math
import random

# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.parent = None  # parent node
        self.cost = 0.0  # cost

# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size

        self.start = Node(start[0], start[1])  # start node
        self.goal = Node(goal[0], goal[1])  # goal node
        self.vertices = []  # list of nodes
        self.found = False  # found flag

        # Parameters for RRT
        self.grid_resolution = 0.1
        self.goal_bias_param = 0.01
        self.goal_distance = 20
        self.extension_distance = 10

    def init_map(self):
        """Intialize the map before each search"""
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    def dis(self, node1, node2):
        """Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        """
        # ### YOUR CODE HERE ###
        eucledian_distance = math.sqrt(math.pow((node1.row - node2.row), 2) + math.pow((node1.col - node2.col), 2))
        return eucledian_distance
        

    def check_collision(self, node1, node2):
        """Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        """
        # Computing Vertical and Horizontal distance
        d_row, d_col = node2.row - node1.row, node2.col - node1.col

        # Calculate the distance and direction between node1 and node2
        distance = math.sqrt(d_row**2 + d_col**2)
        if distance == 0:
            return True  # Nodes are the same; no collision
        
        # Calculate the step size in x and y directions
        row_step = d_row / distance * self.grid_resolution
        col_step = d_col / distance * self.grid_resolution

        row_pt = node1.row
        col_pt = node1.col

        # Check for collisions at regular intervals
        num_steps = int(distance / self.grid_resolution)
        
        for ii in range(num_steps):
            if self.map_array[int(row_pt)][int(col_pt)] == 0:
                return False  # Collision detected
            row_pt += row_step
            col_pt += col_step
        
        return True  # No collision detected

    def get_new_point(self, goal_bias):
        """Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal
                        instead of a random point

        return:
            point - the new point
        """
        # ### YOUR CODE HERE ###
        random_node = Node(np.random.randint(0, self.size_row), np.random.randint(0, self.size_col))
        point = np.random.choice([self.goal, random_node], p=[goal_bias, 1 - goal_bias])
        return point

        # random_prob = random.random()  # Generate a random probability between 0 and 1
        # if random_prob < goal_bias:
        #     point = self.goal  # Choose the goal
        # else:
        #     point = Node(np.random.randint(0, self.size_row), np.random.randint(0, self.size_col))  # Generate a random point
        # return point
    
    def get_nearest_node(self, point):
        """Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        """
        minimum_distance = math.inf
        nearest_node = None  # Initialize nearest_node to None

        for vertex in self.vertices:
            distance = self.dis(vertex, point)
            if distance < minimum_distance:
                minimum_distance = distance
                nearest_node = vertex

        return nearest_node

    def get_neighbors(self, new_node, neighbor_size):
        """Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - list of neighbors that are within the neighbor distance
        """
        # ### YOUR CODE HERE ###
        neighbors = []
        grid_resolution = 0.1  # Adjust the grid resolution as needed

        # Calculate the grid cell indices for the current node
        row_index = int(new_node.row / grid_resolution)
        col_index = int(new_node.col / grid_resolution)

        # Iterate through the nodes in neighboring grid cells
        for ii in range(-1, 2):
            for jj in range(-1, 2):
                neighbor_row = row_index + ii
                neighbor_col = col_index + jj

                # Check if the grid cell is within the map boundaries
                if (
                    neighbor_row >= 0
                    and neighbor_row < self.size_row
                    and neighbor_col >= 0
                    and neighbor_col < self.size_col
                ):
                    # Iterate through nodes in the current grid cell
                    for vertex in self.grid[neighbor_row][neighbor_col]:
                        if self.dis(vertex, new_node) < neighbor_size:
                            neighbors.append(vertex)

        return neighbors

    
    def extension(self, node1, node2):
    
        d_row = node2.row - node1.row
        d_col = node2.col - node1.col
        mod = self.dis(node1, node2)

        # Calculate the step in the direction of node2
        if mod <= self.extension_distance:
            # If the distance between node1 and node2 is smaller than the extension distance,
            # directly return node2 to avoid overshooting the goal
            new_node = node2
        else:
            row_step = d_row * self.extension_distance / mod
            col_step = d_col * self.extension_distance / mod

            # Calculate the new node's position
            row_pt = node1.row + row_step
            col_pt = node1.col + col_step

            # Ensure the new node is within the map boundaries
            row_pt = max(0, min(row_pt, self.size_row - 1))
            col_pt = max(0, min(col_pt, self.size_col - 1))

            new_node = Node(row_pt, col_pt)
            new_node.parent = node1
            new_node.cost = node1.cost + self.dis(new_node, node1)

        return new_node

        
    def draw_map(self):
        """Visualization of the result"""
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker="o", color="y")
            plt.plot(
                [node.col, node.parent.col],
                [node.row, node.parent.row],
                color="y",
            )

        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot(
                    [cur.col, cur.parent.col],
                    [cur.row, cur.parent.row],
                    color="b",
                )
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker="o", color="b")

        # Draw start and goal
        plt.plot(
            self.start.col, self.start.row, markersize=5, marker="o", color="g"
        )
        plt.plot(
            self.goal.col, self.goal.row, markersize=5, marker="o", color="r"
        )

        # show image
        plt.show()

    def RRT(self, n_pts=1000):
        """RRT main search function
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        In each step, extend a new node if possible,
        and check if reached the goal
        """
        # Remove previous result
        self.init_map()
        # In each step,
        # get a new point,
        # get its nearest node,
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.
        for ii in range(n_pts):
            # Sample a new point
            new_point = self.get_new_point(self.goal_bias_param)

            # Find the nearest node in the tree to the new point
            near_vertex = self.get_nearest_node(new_point)

            # Extend from the nearest node towards the new point
            step_node = self.extension(near_vertex, new_point)

            # Check for collision
            if self.check_collision(near_vertex, step_node):
                # Add the valid extended node to the tree
                step_node.parent = near_vertex
                step_node.cost = near_vertex.cost + self.dis(step_node, near_vertex)
                self.vertices.append(step_node)

                # Check if the extended node is close to the goal
                if self.dis(step_node, self.goal) <= self.goal_distance and self.check_collision(step_node, self.goal):
                    self.found = True
                    self.goal.parent = step_node
                    self.goal.cost = step_node.cost + self.dis(step_node, self.goal)
                    break

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" % steps)
            print("The path length is %.2f" % length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
