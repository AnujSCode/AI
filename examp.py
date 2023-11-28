import numpy as np
import sys
import open3d as o3d
import matplotlib.pyplot as plt

# DStarLite class and State class implementation (copy your existing code here)

# Function to filter ground points in the Lidar data
def filter_ground(cloud, ground_level=0):
    cloud = cloud[cloud[:, 2] > ground_level, :]
    return cloud

# Function to generate a 2D map from Lidar data
def generate_map(flat_cloud, dist_from_origin=80, resolution=80):
    side_length = dist_from_origin * 2
    pixel_size = (side_length / resolution)
    map_array = np.zeros((resolution, resolution))

    for x in range(resolution):
        minX = x * pixel_size - dist_from_origin
        maxX = (x + 1) * pixel_size - dist_from_origin
        isX = np.logical_and(flat_cloud[:, 0] >= minX, flat_cloud[:, 0] < maxX)
        if not np.any(isX):
            continue
        for y in range(resolution):
            minY = y * pixel_size - dist_from_origin
            maxY = (y + 1) * pixel_size - dist_from_origin
            isY = np.logical_and(flat_cloud[:, 1] >= minY, flat_cloud[:, 1] < maxY)
            if np.any(np.logical_and(isX, isY)):
                map_array[x][y] = 1

    return map_array

# Load Lidar data from file
initial_pcd = np.loadtxt('cloud.txt').astype(np.float32)

# Remove ground points
pcd_filtered = filter_ground(initial_pcd, -1.2)

# Remove Z axis
pcd_flat = np.delete(pcd_filtered, 2, axis=1)

# Generate a 2D map from the Lidar data
environment_map = generate_map(pcd_flat)

# Define start and goal states based on the map
start_state = State(index=(10, 10))  # Example starting point in pixel coordinates
goal_state = State(index=(70, 70))   # Example goal point in pixel coordinates

# Initialize DStarLite with the generated map
dstar = DStarLite(start_state, goal_state, environment_map)

# Main loop
while True:
    # Simulate Lidar data update or wait for real-world events
    # For simplicity, let's assume the environment changes dynamically
    update_x, update_y = 30, 30  # Example change in the environment
    pcd_update = np.array([[update_x, update_y, 1.0]])  # Example new obstacle
    pcd_flat = np.vstack([pcd_flat, pcd_update])  # Update the Lidar data

    # Update the 2D map with the new Lidar data
    environment_map = generate_map(pcd_flat)

    # Trigger replanning
    dstar.replan()

    # Extract the planned path
    planned_path = dstar.extract_path()

    # Execute the first action of the planned path (move to the next state)
    if len(planned_path) > 1:
        next_state = planned_path[1]
        # Perform actions based on the planned path, e.g., move to next_state
        print(f"Move from {planned_path[0].index} to {next_state.index}")

    # Simulate time passing or wait for real-world events
    plt.imshow(environment_map)
    plt.scatter(pcd_flat[:, 0], pcd_flat[:, 1], s=1, c='red', marker='x')  # Plot Lidar points
    plt.show()
    plt.pause(1)  # Pause to visualize the changes

    # Repeat the loop for the next iteration