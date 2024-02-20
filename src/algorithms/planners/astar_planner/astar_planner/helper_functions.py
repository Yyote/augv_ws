
#!/usr/bin/env python3
from nav_msgs.msg import OccupancyGrid


def get_index_in_map_from_world_coords(x, y, grid_map_: OccupancyGrid):
    """
    Returns index of point in world map to occupancy_grid
    """
    xm, ym = world_to_map(x, y, grid_map_)
    return get_index(xm, ym, grid_map_)


def world_to_map(x: int, y: int, grid_map_: OccupancyGrid):
    """
    Get map coordinates
    """
    mx = int((x - grid_map_.info.origin.position.x) / grid_map_.info.resolution)
    my = int((y - grid_map_.info.origin.position.y) / grid_map_.info.resolution)
    return [mx, my]


def get_index(x: int, y: int, grid_map_: OccupancyGrid):
    """
    Get index in map
    """
    return x + y * grid_map_.info.width

