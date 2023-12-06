from queue import PriorityQueue
import numpy as np

class DStarLite:
    def __init__(self, start, goal, grid):
        self.start = start
        self.goal = goal
        self.grid = grid
        # Initialize other necessary variables here

    def getNeighbors(self, current):
        x, y = current
        # Define the logic to get neighbors based on your grid
        # Example: getting 4-connectivity neighbors
        neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
        valid_neighbors = []

        for neighbor in neighbors:
            neighbor_x, neighbor_y = neighbor
            # Check if the neighbor is within the grid boundaries
            if 0 <= neighbor_x < len(self.grid) and 0 <= neighbor_y < len(self.grid[0]):
                # Check if the neighbor is accessible (you might have specific criteria)
                if self.grid[neighbor_x][neighbor_y] == 0:  # Change this condition as needed
                    valid_neighbors.append(neighbor)

        return valid_neighbors

    def heuristic(self, cell):
        # Implement your heuristic function here
        # Example: Euclidean distance heuristic
        start_x, start_y = self.start
        goal_x, goal_y = self.goal
        cell_x, cell_y = cell

        # Calculate Euclidean distance as heuristic
        heuristic_cost = ((goal_x - cell_x) ** 2 + (goal_y - cell_y) ** 2) ** 0.5

        return heuristic_cost

    def remove(self, current):
        # Flag a cell as removed in the grid (assuming grid is a list of lists)
        x, y = current
        if 0 <= x < len(self.grid) and 0 <= y < len(self.grid[0]):
            self.grid[x][y] = 1  # Modify the value according to your grid representation to indicate removed cell
        else:
            print("Cell coordinates out of range")

    def cost(self, current, neighbor):
        # Example: Euclidean distance as the cost function
        current_x, current_y = current
        neighbor_x, neighbor_y = neighbor

        # Euclidean distance as the cost (you can use other methods based on your needs)
        cost = ((neighbor_x - current_x) ** 2 + (neighbor_y - current_y) ** 2) ** 0.5

        return cost

    def computeShortestPath(self):
        # Initialize open list with the start cell
        open_list = PriorityQueue()
        open_list.put(self.start, 0)

        # Initialize cost and backpointer dictionaries
        g_values = {self.start: 0}
        rhs_values = {self.start: self.heuristic(self.start)}
        backpointers = {}
        removed = set()  # Set to keep track of removed elements

        while not open_list.empty():
            # Get the cell in open list with the lowest rhs value
            current = open_list.get()

            # If the cell is not yet expanded (i.e., g-value and rhs-value are not equal)
            if current in removed:
                continue  # Skip removed elements

            if g_values[current] > rhs_values[current]:
                g_values[current] = rhs_values[current]

                # Simulate removal by re-adding the element with a higher priority
                open_list.put(current, rhs_values[current])

            # For each neighbor of the current cell
            for neighbor in self.getNeighbors(current):
                if neighbor not in g_values:
                    g_values[neighbor] = float('inf')

                # If the cost through the current cell is less than the current cost
                if g_values[neighbor] > g_values[current] + self.cost(current, neighbor):
                    backpointers[neighbor] = current
                    rhs_values[neighbor] = g_values[current] + self.cost(current, neighbor)
                    open_list.put(neighbor, rhs_values[neighbor])

        # Reconstruct the path from the goal to the start
        path = []
        current = self.goal
        while current != self.start:
            path.append(current)
            current = backpointers[current]
        path.append(self.start)  # Optional
        path.reverse()  # Optional: reverse the path to start-to-goal order

        return path

    def getNextInPath(self, current):
        # Get the neighbors of the current cell
        neighbors = self.getNeighbors(current)

        # Initialize the next cell as None and the minimum cost as infinity
        next_cell = None
        min_cost = float('inf')

        # For each neighbor
        for neighbor in neighbors:
            # Calculate the cost from the current cell to the neighbor
            cost = self.cost(current, neighbor)

            # If the cost is less than the minimum cost, update the next cell and the minimum cost
            if cost < min_cost:
                next_cell = neighbor
                min_cost = cost

        # Return the next cell in the path
        return next_cell

    def gridHasChanged(self):
        # Store a copy of the grid at the time of path computation
        if not hasattr(self, 'old_grid'):
            self.old_grid = self.grid.copy()
            return False

        # Check if the grid has changed
        if np.array_equal(self.grid, self.old_grid):
            return False
        else:
            # Update the old grid to the current grid
            self.old_grid = self.grid.copy()
            return True

    def updateGrid(self, grid):
        # Update the internal representation of the grid
        self.grid = grid



def getNeighborsAll(grid, current, q):

    for i in range(current[0]-1, current[0]+2):
        if not 0 <= i < len(grid): continue #if out of range of grid

        for j in range(current[1]-1, current[1]+2):

            if not 0 <= j < len(grid): continue  # if out of range of grid

            if grid[i][j] == 0 and (i,j) != current: # if passable and untouched
                grid[i][j] = current # back pointer
                q.put((i, j))

    return q, grid

def getNeighborsCardinal(grid, current, q):
    points = [(current[0]-1,current[1]), (current[0]+1,current[1]),
              (current[0],current[1]-1), (current[0],current[1]+1)]
    for i,j in points:
        if not 0 <= i < len(grid): continue #if out of range of grid

        if not 0 <= j < len(grid): continue  # if out of range of grid

        if grid[i][j] == 0 and (i,j) != current: # if passable and untouched
            grid[i][j] = current # back pointer
            q.put((i, j))

    return q, grid


def dStarLitePath(gridWorld, start, goal):
    grid = gridWorld

    # Initialize D* Lite
    dstar = DStarLite(start, goal, grid)

    # Compute initial plan
    dstar.computeShortestPath()

    path = []
    current = start
    while current != goal:
        # Get next cell in the path
        current = dstar.getNextInPath(current)

        # If no path is found
        if current is None:
            return -1

        # Add current cell to the path
        path.append(current)

        # If the grid has changed, update the path
        if dstar.gridHasChanged():
            dstar.updateGrid(grid)
            dstar.computeShortestPath()

    return path

