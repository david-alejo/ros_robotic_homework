"""

Grid based Dijkstra planning

author: Atsushi Sakai(@Atsushi_twi)
author: (adaption to ROS by David Alejo)

"""

import math
from nav_msgs.msg import OccupancyGrid

show_animation = False # Put to true to get the animation (could be useful for debugging)

class Dijkstra:
    def __init__(self, costmap):
        """ 
        Initialize a map from a ROS costmap
        
        costmap: ROS costmap
        """
        # Copy the map metadata
        self.resolution = costmap.info.resolution
        self.min_x = costmap.info.origin.position.x
        self.min_y = costmap.info.origin.position.y
        self.y_width = costmap.info.height
        self.x_width = costmap.info.width
        self.max_x = self.min_x + self.x_width *self.resolution
        self.max_y = self.min_y + self.y_width *self.resolution
        print(self.min_x, self.min_y)
        print(self.max_x, self.max_y)
        print("Resolution: ", self.resolution)
        print(self.x_width, self.y_width)
        

        self.motion = self.get_motion_model()
        
        # Copy the actual map data from the map
        x = 0
        y = 0
        ox = list()
        oy = list()
         # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        obstacles = 0
        for value in costmap.data:
            if value > 80:            # This 80 value could change depending on the map
                obstacles += 1
                self.obstacle_map[x][y] = True
                ox.append(float(x)*self.resolution +self.min_x)
                oy.append(float(y)*self.resolution +self.min_y)
            # Update the iterators
            x += 1
            if x == self.x_width:
                x = 0
                y += 1
        print("Loaded %d obstacles"%(obstacles))
  

    class Node:
        def __init__(self, x, y, cost, parent):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost   # TODO: In A* we should differenciate different costs: this should be the f cost
                               # cost from origin
                               # You should also include the g cost (heuristic) and the h cost (total cost)
            self.parent = parent  # index of the previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent)

    def planning(self, sx, sy, gx, gy):
        """
        dijkstra path search

        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                            self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                            self.calc_xy_index(gy, self.min_y), 0.0, -1)
        if (not self.verify_node(start_node)):
            print("Error: init not valid")
            return (0,0)
        
        if (not self.verify_node(goal_node)):
            print("Error: goal not valid")
            return (0,0)
        
        
        print(start_node, goal_node)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node

        while 1:
            c_id = min(open_set, key=lambda o: open_set[o].cost) 
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Goal found")
                goal_node.parent = current.parent
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand search grid based on motion model
            for move_x, move_y, move_cost in self.motion:
                # TODO: You could change the cost of the node to include a heuristic to the goal node
                # This way, we can change from Djikstra to A* algorithm 
                # Currently the cost of the node only takes into account the
                node = self.Node(current.x + move_x,
                                current.y + move_y,
                                current.cost + move_cost, c_id)
                n_id = self.calc_index(node)

                if n_id in closed_set:
                    continue

                if not self.verify_node(node):
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # Discover a new node
                else:
                    if open_set[n_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        
        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [
            self.calc_position(goal_node.y, self.min_y)]
        parent = goal_node.parent
        while parent != -1:
            n = closed_set[parent]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent = n.parent
        rx.reverse() # Do not forget to reverse it
        ry.reverse()
        return rx, ry

    def calc_position(self, index, minp):
        pos = index * self.resolution + minp
        return pos

    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.resolution)

    def calc_index(self, node):
        return (node.x - self.min_x) * self.y_width + (node.y - self.min_y)

    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False
        # print("Reaching node", node.x, node.y)

        if self.obstacle_map[int(node.x)][int(node.y)]:
            return False

        return True

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        # TODO 2: You could include further motion so that we can derive to Theta* algorithm
        # TODO 2: Note that in those cases, we should check for line of sight (when the two nodes are far apart)
        motion = [[1, 0, 1],
                [0, 1, 1],
                [-1, 0, 1],
                [0, -1, 1],
                [-1, -1, math.sqrt(2)],
                [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)],
                [1, 1, math.sqrt(2)]]

        return motion


