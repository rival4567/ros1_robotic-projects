from floodfill_h import *
from basic_h import *
from pose_h import *
from logic.walls_h import *
from logic.grid_h import *


def get_best_cell(x, y, right_node, front_node, left_node, is_virtual=False):
    global pose, grid, walls
    min_val = grid.grid[
        pose.position]  # Initial min value is current tile value
    min_pos = pose.position  # Initial min value position is current tile position

    # Store <x, y> positions of nodes in 4 DIR_TO_ANGLE
    node_xs = [x - 1, x + 1, x, x]
    node_ys = [y, y, y - 1, y + 1]

    is_wall_in_right_val = False
    is_wall_in_left_val = False
    is_wall_in_front_val = False

    if (is_virtual):
        front_wall_dir = orientation_to_wall_direction(pose.orientation())
        right_wall_dir = orientation_to_wall_direction(
            pose.clockwise_direction())
        left_wall_dir = orientation_to_wall_direction(
            pose.counter_clockwise_direction())
        is_wall_in_right_val = walls.has_wall(pose.position, right_wall_dir)
        is_wall_in_left_val = walls.has_wall(pose.position, left_wall_dir)
        is_wall_in_front_val = walls.has_wall(pose.position, front_wall_dir)
        if (print_debug):
            print(pose.position, '|', pose.orientation, '=')
            print(is_wall_in_right_val, is_wall_in_front_val,
                  is_wall_in_left_val)
    else:
        """need to check this"""
        global maze_walls
        is_wall_in_right_val = is_wall_in_right(maze_walls)
        is_wall_in_front_val = is_wall_in_front(maze_walls)
        is_wall_in_left_val = is_wall_in_left(maze_walls)

    for i in range(4):
        node_x = node_xs[i]
        node_y = node_ys[i]
        node = get_position(node_x, node_y)

        if (is_valid_cell(node_x, node_y)
                and (not (node == right_node and is_wall_in_right_val))
                and (not (node == left_node and is_wall_in_left_val))
                and (not (node == front_node and is_wall_in_front_val))):
            val = grid.grid[node]
            if (val < min_val):
                min_val = val
                min_pos = node

    if (flood_fill_priority == 0):
        ## FLR
        if (not is_wall_in_front_val and min_val == grid.grid[front_node]):
            min_pos = front_node
        elif (not is_wall_in_left_val and min_val == grid.grid[left_node]):
            min_pos = left_node
        elif (not is_wall_in_right_val and min_val == grid.grid[right_node]):
            min_pos = right_node
    elif (flood_fill_priority == 1):
        ## FRL
        if (not is_wall_in_front_val and min_val == grid.grid[front_node]):
            min_pos = front_node
        elif (not is_wall_in_right_val and min_val == grid.grid[right_node]):
            min_pos = right_node
        elif (not is_wall_in_left_val and min_val == grid.grid[left_node]):
            min_pos = left_node
    elif (flood_fill_priority == 2):
        ## RFL
        if (not is_wall_in_right_val and min_val == grid.grid[right_node]):
            min_pos = right_node
        elif (not is_wall_in_front_val and min_val == grid.grid[front_node]):
            min_pos = front_node
        elif (not is_wall_in_left_val and min_val == grid.grid[left_node]):
            min_pos = left_node

    return min_pos


def go_to_best_cell():
    global pose, grid, walls
    global is_predicted_node_in_front
    grid.visited[pose.position] = True
    x = get_x(pose.position)
    y = get_y(pose.position)

    right_node = pose.position_of_relative_tile(pose.clockwise_direction())
    front_node = pose.position_of_relative_tile(pose.orientation)
    left_node = pose.position_of_relative_tile(
        pose.counter_clockwise_direction())

    min_pos = get_best_cell(x, y, right_node, front_node, left_node)

    # if (grid.visited[min_pos] and min_pos == front_node):
    #     pose.go_to_tile_in_front()
    #     x_ = get_x(pose.position)
    #     y_ = get_y(pose.position)
    #     right_node_ = pose.position_of_relative_tile(
    #         pose.clockwise_direction())
    #     front_node_ = pose.position_of_relative_tile(pose.orientation)
    #     left_node_ = pose.position_of_relative_tile(
    #         pose.counter_clockwise_direction())

        # next_best_cell = get_best_cell(x_, y_, right_node_, front_node_,
        #                                left_node_)

        # if (next_best_cell == front_node_):
        #     is_predicted_node_in_front = True

        # pose.turn_counter_clockwise()
        # pose.turn_counter_clockwise()
        # pose.go_to_tile_in_front()
        # pose.turn_counter_clockwise()
        #pose.turn_counter_clockwise()

    if (min_pos == right_node):
        go_to_right()
    elif (min_pos == front_node):
        go_forward()
        """optional preparation to fastrun"""
        is_predicted_node_in_front = False
    elif (min_pos == left_node):
        go_to_left()
    else:
        go_backward()

    return min_pos
