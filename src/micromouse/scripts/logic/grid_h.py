from floodfill_h import *
from pose_h import *
from logic.walls_h import *
from collections import deque

UNVISITED = 1000000


class Grid:
    global pose, walls

    def __init__(self):
        self.grid = [UNVISITED] * N_CELLS
        self.visited = [False] * N_CELLS

    def flood_fill(self, search_pos, check_visited=False):
        self.grid = [UNVISITED] * N_CELLS
        queue = deque()

        if (search_pos == CENTER_MAZE):
            queue.append(CENTER_CELL_1)
            queue.append(CENTER_CELL_2)
            queue.append(CENTER_CELL_3)
            queue.append(CENTER_CELL_4)
            self.grid[CENTER_CELL_1] = 0
            self.grid[CENTER_CELL_2] = 0
            self.grid[CENTER_CELL_3] = 0
            self.grid[CENTER_CELL_4] = 0
        else:
            queue.append(search_pos)
            self.grid[search_pos] = 0

        while (queue):
            current = queue.popleft()
            current_x = get_x(current)
            current_y = get_y(current)

            next_node_xs = [current_x - 1, current_x + 1, current_x, current_x]
            next_node_ys = [current_y, current_y, current_y - 1, current_y + 1]
            wall_direction = [WALL_WEST, WALL_EAST, WALL_NORTH, WALL_SOUTH]

            for i in range(4):
                next_node_x = next_node_xs[i]
                next_node_y = next_node_ys[i]
                next_node = get_position(next_node_x, next_node_y)
                if (is_valid_cell(next_node_x, next_node_y)
                        and self.grid[next_node] == UNVISITED
                        and (walls.walls[current] % wall_direction[i] != 0)
                        and (not check_visited or self.visited[next_node])):
                    self.grid[next_node] = self.grid[current] + 1
                    queue.append(next_node)


grid = Grid()