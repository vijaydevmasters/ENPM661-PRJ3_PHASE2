# Github link: https://github.com/SYED-ABRARUDDIN/ENPM661_PROJECT3

import numpy as np
import cv2
import math
from queue import PriorityQueue
# Function to map coordinates to the bottom left of the image
def map_to_bottom_left(point):
    """
    Maps the given coordinates to the bottom left corner of a rectangle.

    Parameters:
    x (int): The x-coordinate of the point.
    y (int): The y-coordinate of the point.
    width (int): The width of the rectangle.
    height (int): The height of the rectangle.

    Returns:
    tuple: A tuple containing the x and y coordinates of the point mapped to the bottom left corner.
    """
    height, width = 500, 1200
    bottom_left_x = point[0]
    bottom_left_y = height - point[1]
    return (bottom_left_x, bottom_left_y)


# Function to check if a point is a valid neighbor
def is_valid_neighbor(point, obstacles):
    """
    Checks if a given point is a valid neighbor based on the obstacle map.

    Args:
        point (tuple): The coordinates of the point to check.
        obstacles (numpy.ndarray): The obstacle map.

    Returns:
        bool: True if the point is a valid neighbor, False otherwise.
    """
    x, y,_ = point
    if 0 <= x < width and 0 <= y < height and obstacle_map[int(y), int(x), 0] == 255:
        return True
    return False

def get_neighbors(point, obstacles, UL = 5, UR = 10):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    neighbors = []
    Xi, Yi, Thetai = point
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180
    actions=[[UL,UR], [UR,UL],[0,0],[UL,0],[UR,0],[0,UR],[0,UL], [UL,UL]]

    # Xi, Yi,Thetai: Input point's coordinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, Thetan: End point coordintes
    for action in actions:
        UL = action[0]
        UR = action[1]
        D=0
        Xs = Xn
        Ys = Yn
        while t<1:
            t = t + dt

            Delta_Xn = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
            Delta_Yn = 0.5*r * (UL + UR) * math.sin(Thetan) * dt
            Thetan += (r / L) * (UR - UL) * dt

            Xs+=Delta_Xn
            Ys+=Delta_Yn

        Thetan = 180 * (Thetan) / 3.14

        neighbor = (Xs,Ys-1,Thetan)
        print(neighbor)
        if is_valid_neighbor(neighbor, obstacles):
            print(neighbor)
            cv2.circle(obstacle_map, (int(Xn),int(Yn)), 3,(120,244,255), -1 )
            cv2.imshow("Shortest Path", obstacle_map)
            cv2.waitKey(1)
            neighbors.append(neighbor)
             
    return neighbors
    
  
# def get_neighbors(point, obstacles):
#     """
#     Returns a list of valid neighboring points along with their orientations relative to the current point.

#     Args:
#         point (tuple): The coordinates and orientation (x, y, theta) of the point.
#         obstacles (list): A list of coordinates representing obstacles.

#     Returns:
#         list: A list of tuples containing valid neighboring points and their orientations.

#     """
#     global print_interval
#     x, y, theta = point
#     neighbors = []
#     angles = [60, 30, 0, -30, -60]
#     distance = step_size
#     for angle in angles:
#         # Calculate absolute angle for the neighbor
#         neighbor_angle = (theta + angle) % 360
#         # Convert angle from degrees to radians
#         angle_rad = math.radians(neighbor_angle)
#         # Calculate new neighbor coordinates
#         nx = x + (distance * math.cos(angle_rad))
#         ny = y + (distance * math.sin(angle_rad))
#         #print("nx: "+str(nx)+" ny: "+str(ny))
#         neighbor = (int(math.floor(nx)), int(math.floor(ny)), neighbor_angle)  # Round to nearest integer
#         if is_valid_neighbor(neighbor, obstacles):
#             cv2.line(obstacle_map, (x,y),(int(math.floor(nx)),int(math.floor(ny))),(0,0,255),1)
#             if print_interval%500==0:
#                 cv2.imshow("Shortest Path", obstacle_map)
#                 out.write(obstacle_map)
#                 cv2.waitKey(1)
#             #print("neighbor: "+str(neighbor))   
#             print_interval=print_interval+1

#             neighbors.append(neighbor)
#     return neighbors
# Function to ask for a point from the user


# Function to draw a line on the image
def draw_line(img, start_point, end_point, color, thickness):
    cv2.line(img, start_point, end_point, color, thickness)

# Function to draw obstacles using half-plane equations
def draw_obstacles(obstacle_map, obstacles):
    """
    Draw obstacles on the given obstacle map.

    Parameters:
    - obstacle_map: The map on which obstacles will be drawn.
    - obstacles: A list of dictionaries representing the obstacles. Each dictionary should have the following keys:
        - 'vertices': A list of vertices (points) that define the shape of the obstacle.
        - 'color' (optional): The color of the obstacle. Default is black.
        - 'thickness' (optional): The thickness of the lines used to draw the obstacle. Default is 1.

    Returns:
    None
    """

    for obstacle in obstacles:
        color = obstacle.get('color', (0, 0, 0))  # Default color is black
        thickness = obstacle.get('thickness', 1)  # Default thickness is 1

        vertices = obstacle['vertices']
        for i in range(len(vertices)):
            draw_line(obstacle_map, map_to_bottom_left(vertices[i]), map_to_bottom_left(vertices[(i + 1) % len(vertices)]), color, thickness)

        cv2.fillPoly(obstacle_map, np.array([[map_to_bottom_left(point) for point in vertices]]), color)


# Function to calculate the Euclidean distance between two points
def euclidean_distance(p1, p2):
    """
    Calculates the Euclidean distance between two points in a two-dimensional space.
    
    Parameters:
        p1 (tuple): The coordinates of the first point (x1, y1).
        p2 (tuple): The coordinates of the second point (x2, y2).
        
    Returns:
        float: The Euclidean distance between the two points.
    """
    
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def heuristic(node, goal):
    """
    Calculates the heuristic value between a given node and the goal node.
    
    Parameters:
        node (tuple): The coordinates and angle of the current node.
        goal (tuple): The coordinates and angle of the goal node.
    
    Returns:
        float: The heuristic value between the current node and the goal node.
    """
    x1, y1, theta1 = node
    x2, y2, theta2 = goal
 
    return euclidean_distance((x1, y1), (x2, y2))


# Function to calculate the cost of moving from one point to another
def distance_cost(current, next):
    """
    Calculates the cost of moving from the current node to the next node.
    
    Parameters:
        current (tuple): The coordinates of the current node.
        next (tuple): The coordinates of the next node.
    
    Returns:
        float: The cost of moving from the current node to the next node.
    """
    

    return euclidean_distance(current, next)

def ask_for_point(message, default=None):
    """
    Asks the user to input a point and validates its validity based on the obstacle map.

    Args:
        message (str): The message to display when asking for the point.
        default (tuple, optional): The default point to use if the user does not provide any input. 
                                   Defaults to None.

    Returns:
        tuple: The validated point (x, y).

    Raises:
        ValueError: If no default value is provided and the user does not provide any input.
    """
    while True:
        user_input = input(f"{message} (default: {default[0]},{default[1]},{default[2]}): ")
        if user_input.strip() == "":
            if default is None:
                raise ValueError("No default value provided.")
            else:
                x, y,theta = default
                x, y = map_to_bottom_left((x, y))
        else:
            x, y, theta = map(int, user_input.split(','))
            x, y = map_to_bottom_left((x, y))

        if 0 <= x < width and 0 <= y < height and obstacle_map[y, x, 0] == 255 and theta%30==0 and theta%30 == 0:
            return x, y,theta
        elif theta%30 !=0:
            print("Enter angle in multiples of 30 degrees")
        
        else:
            print("Point is invalid.")
def ask_clearence(radius):
    print("Click ENTER for entering default value ")
    while True:
        user_input = input("Enter by how much the obstacles and map walls need to be bloated (default 5): ")
        if not user_input:  # If the user just clicks enter, use the default value
            return 5
        try:
            clearance = int(user_input)
            if clearance > 0:
                return clearance
            else:
                print("Enter a positive value for clearance")
        except ValueError:
            print("Invalid input. Please enter a number or press Enter for default.")

def ask_robot_radius():
    while True:
        user_input = input("Enter Robot Radius (default 5): ")
        if not user_input:  # If the user just clicks enter, use the default value
            return 5
        try:
            radius = int(user_input)
            if radius > 0:
                return radius
            else:
                print("Enter a positive value for radius")
        except ValueError:
            print("Invalid input. Please enter a number or press Enter for default.")

def ask_step_size():
    while True:
        user_input = input("Enter Step Size (1 to 10, default 5): ")
        if not user_input:  # If the user just clicks enter, use the default value
            return 5
        try:
            step_size = int(user_input)
            if 1 <= step_size <= 10:
                return step_size
            else:
                print("Enter a value for Step Size in the range 1 to 10")
        except ValueError:
            print("Invalid input. Please enter a number or press Enter for default.")


def a_star(start, goal, obstacles, threshold):
    """
    A* algorithm implementation to find the shortest path from start to goal.

    Parameters:
    - start: Tuple representing the start node coordinates (x, y, theta).
    - goal: Tuple representing the goal node coordinates (x, y, theta).
    - obstacles: List of obstacles in the environment.
    - threshold: Threshold value for considering the goal reached.
    - step_size: Step size for generating neighboring nodes.

    Returns:
    - path: List of nodes representing the shortest path from start to goal.
    """

    frontier = PriorityQueue()
    frontier.put((0, start))

    cost_so_far = {start: 0}
    came_from = {start: None}
    flag = 0

    while not frontier.empty():
        current_cost, current_node = frontier.get()

        if (current_node[0] > goal[0] - threshold and current_node[0] < goal[0] + threshold) and (current_node[1] > goal[1] - threshold and current_node[1] < goal[1] + threshold):
            
            print("Goal Threshold reached")

        for next_node in get_neighbors(current_node, obstacles):
            new_cost = cost_so_far[current_node] + distance_cost(current_node, next_node)
            new_cost_check = new_cost + heuristic(next_node, goal)
            
            if next_node not in cost_so_far or new_cost_check < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                if flag >= 1:
                    priority = flag
                else:
                    priority = round(new_cost + heuristic(next_node, goal), 3)  # A* uses f = g + h
                frontier.put((priority, next_node))
                came_from[next_node] = current_node

    path = []
    while current_node != start:
        path.append((current_node,came_from[current_node]))
        current_node = came_from[current_node]
    path.reverse()

    return path





# Define image dimensions
width = 1200
height = 500

# Create a blank image filled with white
obstacle_map = np.ones((height, width, 3), dtype=np.uint8) * 255



Robot_radius = ask_robot_radius()
clearance = ask_clearence(Robot_radius)
# step_size= ask_step_size()



print_interval=0
clearance=clearance+Robot_radius

obstacles = [
    {'shape': 'rectangle', 'vertices': [(100-clearance, 100-clearance), (100-clearance, 500+clearance), (175+clearance, 500+clearance), (175+clearance, 100-clearance)], 'color': (128, 128, 128), 'thickness': 1},  # Rectangle obstacle 2
    {'shape': 'rectangle', 'vertices': [(100, 100), (100, 500), (175, 500), (175, 100)], 'color': (0, 0, 0), 'thickness': 1},  # Rectangle obstacle 2
    {'shape': 'rectangle', 'vertices': [(275-clearance, 0), (275-clearance, 400+clearance), (350+clearance, 400+clearance), (350+clearance, 0)], 'color': (128, 128, 128), 'thickness': 1},  # Rectangle obstacle 3
    {'shape': 'rectangle', 'vertices': [(275, 0), (275, 400), (350, 400), (350, 0)], 'color': (0, 0, 0), 'thickness': 1},  # Rectangle obstacle 4
    {'shape': 'hexagon', 'vertices': [(650, 400),(520, 325),(520, 175),(650, 100),(780, 175),(780, 325)], 'color': (128, 128, 128), 'thickness': clearance-1}, 
    {'shape': 'hexagon', 'vertices': [(650, 395),(524, 323),(524, 178),(650, 105),(776, 177),(776, 322)], 'color': (0, 0, 0), 'thickness': 1},  # Hexagon obstacle
    {'shape': 'rectangle', 'vertices': [(1025-clearance, 50-clearance), (1025-clearance, 450+clearance), (1100+clearance, 450+clearance), (1100+clearance, 50-clearance)], 'thickness': 1, 'color': (128, 128, 128)},  # Rectangle obstacle 5
    {'shape': 'rectangle', 'vertices': [(900-clearance, 50-clearance), (900-clearance, 125+clearance), (1100+clearance, 125+clearance), (1100+clearance, 50-clearance)], 'thickness': 1, 'color': (128, 128, 128)},  # Rectangle obstacle 6
    {'shape': 'rectangle', 'vertices': [(900, 50), (900, 125), (1100, 125), (1100, 50)], 'thickness': 1, 'color': (0, 0, 0)},  # Rectangle obstacle 7
    {'shape': 'rectangle', 'vertices': [(900-clearance, 375-clearance), (900-clearance, 450+clearance), (1100+clearance, 450+clearance), (1100+clearance, 375-clearance)], 'thickness': 1, 'color': (128, 128, 128)},  # Rectangle obstacle 8
    {'shape': 'rectangle', 'vertices': [(900, 375), (900, 450), (1100, 450), (1100, 375)], 'thickness': 1, 'color': (0, 0, 0)},  # Rectangle obstacle 9
    {'shape': 'rectangle', 'vertices': [(1025, 50), (1025, 450), (1100, 450), (1100, 50)], 'thickness': 1, 'color': (0, 0, 0)},  # Rectangle obstacle 10
    {'shape': 'rectangle', 'vertices': [(0, 0), (0, clearance), (1200, clearance), (1200, 0)], 'color': (128, 128, 128), 'thickness': 1},  # Rectangle obstacle 11
    {'shape': 'rectangle', 'vertices': [(0, 0), (0, 500), (clearance, 500), (clearance, 0)], 'color': (128, 128, 128), 'thickness': 1},  # Rectangle obstacle 12
    {'shape': 'rectangle', 'vertices': [(1200-clearance, 0), (1200-clearance, 500), (1200, 500), (1200, 0)], 'color': (128, 128, 128), 'thickness': 1},  # Rectangle obstacle 13
    {'shape': 'rectangle', 'vertices': [(0, 500-clearance), (0, 500), (1200, 500), (1200, 500-clearance)], 'color': (128, 128, 128), 'thickness': 1},  # Rectangle obstacle 14
]

# Draw obstacles on the obstacle map
draw_obstacles(obstacle_map, obstacles)

# Ask for start and end points
start = ask_for_point("Enter start point (x, y,theta): ", (50, 50,0))
goal = ask_for_point("Enter goal point (x, y,theta): ", (1150, 50,30))
cv2.circle(obstacle_map, (start[0], start[1]), 5, (255, 0, 0), -1)  # Explored nodes in green
cv2.circle(obstacle_map, (goal[0], goal[1]), 3, (0, 0, 255), -1)  # Explored nodes in green



fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# Save the obstacle map with the shortest path as a video
out = cv2.VideoWriter('Shortest_Path.mp4', fourcc, 60.0, (width, height))

threshold =int( (1.5*Robot_radius)+(10))
cv2.circle(obstacle_map, (goal[0], goal[1]), threshold, (0, 0, 255), 1)  # Explored nodes in green
# Find the shortest path using Dijkstra's algorithm
shortest_path = a_star(start, goal, obstacles,threshold)


# Mark the shortest path on the obstacle map
for point in shortest_path:

    cv2.line(obstacle_map, (point[0][0],point[0][1]), (point[1][0],point[1][1]), (255, 0, 0), 2)  # Explored nodes in green
    cv2.imshow("Shortest Path", obstacle_map)
    out.write(obstacle_map)
    cv2.waitKey(1)

# Display the obstacle map with the shortest path
cv2.imshow("Shortest Path", obstacle_map)
out.write(obstacle_map)
out.release()

cv2.waitKey(0)
cv2.destroyAllWindows()