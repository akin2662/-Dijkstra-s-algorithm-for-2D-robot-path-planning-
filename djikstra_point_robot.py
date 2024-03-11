# importing important libraries
import cv2 as cv
import numpy as np
from collections import OrderedDict
import heapq as hq
import time


# Generating the map
def map():

# obstacle colors
    width = 1200
    height = 500
    red = (255, 0, 0)
    green = (0, 255, 0)
    blue = (0, 0, 255)
    yellow = (0, 255, 255)
    white = (255,255,255)


# vertics for the hexagon
    vertices = np.array([[520,155],[650,80],[780,155],[780,305],[650,380],[520,305]],dtype='int32')
    vertices_clr = np.array([[515, 150], [650, 73], [785, 150], [785, 310], [650, 387], [515, 310]], dtype='int32')

# obstacles and the clearance on a black canvas
    image = np.zeros((height,width,3),dtype='uint8')
    cv.rectangle(image, (95, 0), (180, 405), white, thickness=-1)
    cv.rectangle(image, (100, 0), (175, 400), blue, thickness=-1)
    cv.rectangle(image, (270, 95), (355, 500), white, thickness=-1)
    cv.rectangle(image, (275, 100), (350, 500), red, thickness=-1)
    cv.rectangle(image, (895, 45), (1105, 130), white, thickness=-1)
    cv.rectangle(image, (1015, 120), (1105, 380), white, thickness=-1)
    cv.rectangle(image, (895, 370), (1105, 455), white, thickness=-1)
    cv.rectangle(image, (900,50),(1100,125),yellow,thickness=-1)
    cv.rectangle(image, (1020, 125), (1100, 375), yellow, thickness=-1)
    cv.rectangle(image,(900,375),(1100,450),yellow,thickness=-1)
    cv.fillPoly(image,[vertices_clr],white)
    cv.fillPoly(image,[vertices],green)

    return image

# Check if the node is within boundaries
def within_boundary(node):
    if 0 <= node[0] < 500:
        if 0 <= node[1] < 1200:
            return True

    return False

# Check if the node is in freespace
def in_free_space(image):
    free_space = np.where(image == 0)
    free_space_list = list(zip(free_space[0],free_space[1]))
    free_space_dict = OrderedDict.fromkeys(free_space_list)
    return list(free_space_dict)

# Move the robot up
def up(image, node):
    new_node = (node[0] + 1, node[1])
    if within_boundary(new_node) and new_node in in_free_space(image):
        return (new_node,1)

# Move the robot down
def down(image, node):
    new_node = (node[0] - 1, node[1])
    if within_boundary(new_node) and new_node in in_free_space(image):
        return (new_node,1)

# Move the robot to the left
def left(image, node):
    new_node = (node[0], node[1] - 1)
    if within_boundary(new_node) and new_node in in_free_space(image):
        return (new_node,1)

# Move the robot to the right
def right(image, node):
    new_node = (node[0], node[1] + 1)
    if within_boundary(new_node) and new_node in in_free_space(image):
        return (new_node,1)


# Move the robot up diagonally to the right
def up_right(image, node):
    new_node = (node[0] + 1, node[1] + 1)
    if within_boundary(new_node) and new_node in in_free_space(image):
        return (new_node,np.sqrt(2))

# Move the robot up diagonally to the left
def up_left(image, node):
    new_node = (node[0] + 1, node[1] - 1)
    if within_boundary(new_node) and new_node in in_free_space(image):
        return (new_node,np.sqrt(2))

# Move the robot down diagonally to the right
def down_right(image, node):
    new_node = (node[0] - 1, node[1] + 1)
    if within_boundary(new_node) and new_node in in_free_space(image):
        return (new_node,np.sqrt(2))

# Move the robot down diagonally to the left
def down_left(image, node):
    new_node = (node[0] - 1, node[1] - 1)
    if within_boundary(new_node) and new_node in in_free_space(image):
        return (new_node,np.sqrt(2))

def performActions(image,node):
    u = up(image, node)
    d = down(image, node)
    l = left(image, node)
    r = right(image, node)
    ur = up_right(image,node)
    ul = up_left(image,node)
    dr = down_right(image,node)
    dl = down_left(image,node)

    return [u,d,l,r,ur,ul,dr,dl]

def backtrack(path,start_node,goal_node):
    node = goal_node
    final_path = []
    while node is not start_node:
        final_path.append(node)
        node = path[node]
    final_path.append(start_node)
    final_path.reverse()
    return final_path

def djikstra(image,start_node,goal_node):
    start_node_cost = 0
    open_list = []
    closed_list = {start_node:start_node_cost}
    path = {}
    hq.heappush(open_list,(start_node_cost,start_node))
    hq.heapify(open_list)
    while open_list:
        current_distance, current_node = hq.heappop(open_list)
        if current_node == goal_node and current_node != start_node:
            break

        neighbors = performActions(image,current_node)

        for neighbor in neighbors:
            if neighbor is not None:
                new_node, new_cost = neighbor
                new_distance = current_distance + new_cost

                if new_node not in closed_list or new_distance< closed_list[new_node] :
                    closed_list[new_node] = new_distance
                    path[new_node] = current_node
                    hq.heappush(open_list, (closed_list[new_node], new_node))

    final_path = backtrack(path,start_node,goal_node)
    return final_path


start_time = time.time()
image = map()
start_y = int(input("Y coordinate of the start position:"))
start_x = int(input("X coordinate of the start position:"))
start_node = (start_y,start_x)

goal_y = int(input("Y coordinate of the goal position"))
goal_x = int(input("X coordinate of the goal position"))
goal_node = (goal_y,goal_x)

path = djikstra(image,start_node,goal_node)
print(path)
end_time = time.time()
print(end_time-start_time)

