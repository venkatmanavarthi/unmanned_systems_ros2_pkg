import random
import math
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y, cost=0, parent_index=-1, g_cost=0, h_cost=0):
        self.x = x
        self.y = y
        self.parent_index = parent_index
        self.cost = cost
        self.g_cost = g_cost
        self.h_cost = h_cost

def compute_index(min_x, max_x, min_y, max_y, grid_spacing, cx, cy) -> int:

    index = ((cx - min_x)/grid_spacing) + \
            (((cy - min_y)/grid_spacing) * \
            (((max_x + grid_spacing)-min_x)/grid_spacing))

    return int(index)

def calculate_distance(x1, y1, x2, y2):
    """
    Calcu
    Args:
      x1: X for input location 1 
      x2: Y for input location 1
      y1: X for input location 2
      y2: Y for input location 2

    Returns:
      The Euclidean distance
    """
    return round(math.sqrt((x2 - x1)**2 + (y2 - y1)**2), 2)
class Obstacle:
    def __init__(self, x, y, radius=0) -> None:
        self.x = x
        self.y = y
        self.radius = radius

    def is_inside(self, cur_x, cur_y, r_radius=0.2) -> bool:
        """
        Args:
          cur_x: X position of the robot
          cur_y: Y potition of the robot
          r_radius: Robot radius

        Returns:
          True if point is inside the obstacle
          False if point is outside the obstacle
        """ 
        dis = calculate_distance(cur_x, cur_y, self.x, self.y)
        if dis > (self.radius + r_radius):
            return False
        return True
    
class RRT:

    def __init__(self, minx, maxx, miny, maxy, gs, obs_list, iterations=1000, robot_radius=0.5) -> None:
        """
        Args:
          iterations: max number of iterations
        Returns:
          None
        """
        self.minx = minx
        self.maxx = maxx
        self.miny = miny
        self.maxy = maxy
        self.gs = gs
        self.obs_list = obs_list
        self.iterations = iterations
        self.sample_hist = []
        self.goal = None
        self.start = None
        self.tree = None
        self.robot_radius = robot_radius


    def path(self, goal_node: Node):
        """
        Args:
          goal_node(Node): goal node
        Returns:
          Full path from start to end
        """
        path_nodes = []
        current_node = goal_node
        while current_node != -1:
            path_nodes.append((current_node.x, current_node.y))
            current_node = current_node.parent_index
        return path_nodes
    
    def is_collision(self, point):
      for obs in self.obs_list:
          if obs.is_inside(cur_x=point.x, cur_y=point.y, r_radius=self.robot_radius):
              return True
      return False
    
    def get_new_node(self, tree, random_point):
        nearest_node = None
        min_dist = float('inf')
        for node in tree:
            d = calculate_distance(x1=node.x, x2=random_point.x, y1=node.y, y2=random_point.y)
            if d < min_dist and not self.is_collision(node):
                nearest_node = node
                min_dist = d

        if nearest_node is not None:
            angle = math.atan2(random_point.y - nearest_node.y, random_point.x - nearest_node.x)
            new_x = int(nearest_node.x + self.gs * math.cos(angle))
            new_y = int(nearest_node.y + self.gs * math.sin(angle))
            new_node = Node(new_x, new_y)
            new_node.parent_index = nearest_node
            return new_node

        return None
    def run(self, start, goal):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        rrt_tree = [self.start]
        for iteration in range(self.iterations):
          print("__it__", iteration)
          random_point = Node(int(random.uniform(self.minx, self.maxx)), int(random.uniform(self.miny, self.maxy)))
          new_node = self.get_new_node(rrt_tree, random_point)
  
          if new_node:
              rrt_tree.append(new_node)
          
          if calculate_distance(x1=new_node.x, y1=new_node.y, x2=self.goal.x, y2=self.goal.y) < self.gs:
              self.goal.parent_index = new_node
              path = self.path(self.goal)
              return path, rrt_tree