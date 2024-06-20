# Basic searching algorithms
from util import Stack, Queue, PriorityQueueWithFunction, PriorityQueue
import numpy as np

# Class for each node in the grid
class Node:
    def __init__(self, row, col, cost, parent, goal):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.cost = cost      # total cost (depend on the algorithm)
        self.goal = goal      # Goal   
        self.parent = parent  # previous node

def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
    and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
    e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]. 
            If no path exists return an empty list [].
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('maps/test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    # define directions.
    directions = [(0,1),(1,0), (0,-1),(-1,0)] # right, down, left, up.

    # getting grid dimensions.
    rows, cols = len(grid), len(grid[0])

    # Initialize a visited grid to keep track of the visited nodes.
    visited_nodes = [[False for _ in range(cols)] for _ in range(rows)]

    # Initialize the Queue for BFS.
    queue = Queue()

    # Initialize the start node and add it to the queue.
    root_node = Node(start[0], start[1], 0, None, False)
    # Enqueue the start node in the Queue.
    queue.push(root_node)

    # Update Variables. Tracking paths and steps.
    path = []
    steps = -1
    found = False

    # Implementing BFS.
    while not queue.isEmpty():

        current_node = queue.pop()
        steps += 1

        # Condition for finding the goal. If the goal is found, backtrack to get the path.
        if current_node.row == goal[0] and current_node.col == goal[1]:

            found = True

            while current_node:
                path.insert(0,[current_node.row, current_node.col])
                current_node = current_node.parent

            break

        # Explore neighbouring nodes. Checking each of the 4 directions that we have specified.
        for drow, dcol in directions:
            new_row, new_col = current_node.row + drow , current_node.col + dcol

            # Check if the new position is within bounds
            if 0 <= new_row < rows and 0 <= new_col < cols:
                if grid[new_row][new_col] == 0 and not visited_nodes[new_row][new_col]:
                    # Create a new node for the neighbor and add it to the queue
                    neighbor_node = Node(new_row, new_col, current_node.cost + 1, current_node, False)
                    queue.push(neighbor_node)
                    visited_nodes[neighbor_node.row][neighbor_node.col] = True

    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps

def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
    and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
    e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]. If no path exists return
            an empty list []
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('maps/test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    # directions
    directions = [[-1,0],[0,-1],[1,0],[0,1]]  # right, left, down, up.

    # Getting grid dimensions.
    rows, cols = len(grid), len(grid[0])

    # Initialize a visited grid to keep track of the visited nodes.
    visited_nodes = [[False for _ in range(cols)] for _ in range(rows)]
    
    # Initialize the Stack for DFS.
    stack = Stack()

    # Initialize the start node and add it to the stack.
    root_node = Node(start[0], start[1], 0, None, False)
    stack.push(root_node)

    # Update Variables. Tracking paths and steps.
    path = []
    steps = 0
    found = False

    # Implementing DFS.
    while not stack.isEmpty():
        current_node = stack.pop()
        visited_nodes[current_node.row][current_node.col] = True
        steps += 1

        # Condition for finding the goal. If the goal is found, backtrack to get the path.
        if current_node.row == goal[0] and current_node.col == goal[1]:
            found = True
            while current_node:
                path.insert(0, [current_node.row, current_node.col])
                current_node = current_node.parent
            break

        # Explore neighboring nodes. Checking each of the 4 directions that we have specified.
        for drow, dcol in directions:
            new_row, new_col = current_node.row + drow, current_node.col + dcol

            # Check if the new position is within bounds
            if (0 <= new_row < rows) and (0 <= new_col < cols):
                if (not visited_nodes[new_row][new_col]) and (grid[new_row][new_col] == 0): 
                    # Create a new node for the neighbor and add it to the stack
                    neighbor_node = Node(new_row, new_col, current_node.cost + 1, current_node, False)
                    stack.push(neighbor_node)
                            
        
    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")

    return path, steps

def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
    and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
    e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]. If no path exists return
            an empty list []
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('maps/test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''

    # Define Manhattan distance heuristic function
    def heuristic(node):
        return abs(node.row - goal[0]) + abs(node.col - goal[1])

    # Define directions
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # right, down, left, up.

    # Getting grid dimensions
    rows, cols = len(grid), len(grid[0])

    # Initialize a visited grid to keep track of the visited nodes
    visited_nodes = [[False for _ in range(cols)] for _ in range(rows)]

    # Initialize the priority queue for A* search 
    priority_queue = PriorityQueueWithFunction(lambda node: node.cost + heuristic(node))

    # Initialize the start node and add it to the priority queue
    root_node = Node(start[0], start[1], 0, None, False)
    priority_queue.push(root_node)

    # Update Variables. Tracking paths and steps.
    path = []
    steps = 0
    found = False

    # Implementing A* search
    while not priority_queue.isEmpty():
        current_node = priority_queue.pop()
        steps += 1

        # Condition for finding the goal. If the goal is found, backtrack to get the path.
        if current_node.row == goal[0] and current_node.col == goal[1]:
            found = True

            while current_node:
                path.insert(0, [current_node.row, current_node.col])
                current_node = current_node.parent

            break

        # Explore neighboring nodes. Checking each of the 4 directions that we have specified.
        for drow, dcol in directions:
            new_row, new_col = current_node.row + drow, current_node.col + dcol

            # Check if the new position is within bounds
            if 0 <= new_row < rows and 0 <= new_col < cols:
                if grid[new_row][new_col] == 0 and not visited_nodes[new_row][new_col]:
                    # Create a new node for the neighbor and add it to the priority queue
                    neighbor_node = Node(new_row, new_col, current_node.cost + 1, current_node, False)
                    priority_queue.push(neighbor_node)
                    visited_nodes[new_row][new_col] = True

    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps

# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
