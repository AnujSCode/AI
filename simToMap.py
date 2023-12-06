import numpy as np
import matplotlib.pyplot as plt


def filter_ground(cloud, ground_level=0):
    cloud = cloud[cloud[:, 2] > ground_level, :]
    return cloud

def generate_map(flat_cloud, size, resolution):
    side_length = size
    pixel_size = (side_length/resolution)
    map = np.zeros((resolution, resolution))

    for x in range(resolution):
        minX= x*pixel_size
        maxX = (x+1)*pixel_size
        isX = np.logical_and(flat_cloud[:,0] >= minX, flat_cloud[:,0] <maxX)
        if not np.any(isX):
            continue
        for y in range(resolution):
            minY = y * pixel_size
            maxY = (y + 1) * pixel_size
            isY = np.logical_and(flat_cloud[:,1] >= minY, flat_cloud[:,1] <maxY)
            if (np.any(np.logical_and(isX, isY))):
                map[x][y] = 1
    # map = np.flip(map, axis=1)
    # map = np.rot90(map)
    return map

