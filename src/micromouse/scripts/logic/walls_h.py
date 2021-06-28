from floodfill_h import *
from basic_h import *
from pose_h import *


class Walls:
    global pose

    def __init__(self):
        self.walls = [1] * N_CELLS
        self.init_walls()

    def init_walls(self):
        for pos in range(N_CELLS):
            x = get_x(pos)
            y = get_y(pos)
            if (x == 0):
                self.add_wall(pos, WALL_WEST)
            if (y == 0):
                self.add_wall(pos, WALL_NORTH)
            if (y == SIDE_SQUARES - 1):
                self.add_wall(pos, WALL_SOUTH)
            if (x == SIDE_SQUARES - 1):
                self.add_wall(pos, WALL_EAST)

    def add_wall(self, position, wall_direction):
        if self.has_no_wall(position, wall_direction):
            self.walls[position] *= wall_direction
            x = get_x(position)
            y = get_y(position)
            if wall_direction == WALL_EAST:
                attached_cell_wall_direction = WALL_WEST
                x += 1
            elif wall_direction == WALL_WEST:
                attached_cell_wall_direction = WALL_EAST
                x -= 1
            elif wall_direction == WALL_NORTH:
                attached_cell_wall_direction = WALL_SOUTH
                y -= 1
            else:
                attached_cell_wall_direction = WALL_NORTH
                y += 1

            attached_cell_position = get_position(x, y)
            if is_valid_cell(x, y) and (self.walls[attached_cell_position] %
                                        attached_cell_wall_direction != 0):
                self.walls[
                    attached_cell_position] *= attached_cell_wall_direction

    def update_walls_from_maze(self):
        global maze_walls

        right = pose.clockwise_direction()
        front = pose.orientation
        left = pose.counter_clockwise_direction()

        direction = [right, front, left]

        has_wall = [
            is_wall_in_right(maze_walls),
            is_wall_in_front(maze_walls),
            is_wall_in_left(maze_walls)
        ]

        for i in range(3):
            if has_wall[i]:
                wall_direction = orientation_to_wall_direction(direction[i])
                self.add_wall(pose.position, wall_direction)

    def has_no_wall(self, position, wall_direction):
        return not self.has_wall(position, wall_direction)

    def has_wall(self, position, wall_direction):
        return self.walls[position] % wall_direction == 0


walls = Walls()
maze_walls = Walls()
