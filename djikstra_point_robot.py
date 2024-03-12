# importing important libraries
import cv2 as cv
import numpy as np
import heapq as hq
import time

#variable to resize the image
resize = 0.25
# Function to take user input
def user_input():
    start_x = int(int(input(f"X coordinate of the start position:"))*resize)
    start_y = int(int(input(f"Y coordinate of the start position:"))*resize)
    start_node = (start_y,start_x)

    goal_x = int(int(input("X coordinate of the goal position"))*resize)
    goal_y = int(int(input("Y coordinate of the goal position"))*resize)
    goal_node = (goal_y,goal_x)
    return start_node,goal_node
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


# vertices for the hexagon
    vertices = np.array([[520,155],[650,80],[780,155],[780,305],[650,380],[520,305]],dtype='int32')
    vertices_clr = np.array([[515, 150], [650, 73], [785, 150], [785, 310], [650, 387], [515, 310]], dtype='int32')

# obstacles and the clearance on a black canvas
    image = np.zeros((height,width,3),dtype='uint8')
    cv.rectangle(image, (95, 500), (180, 95), white, thickness=-1)
    cv.rectangle(image, (100, 500), (175, 100), blue, thickness=-1)
    cv.rectangle(image, (270, 405), (355, 0), white, thickness=-1)
    cv.rectangle(image, (275, 400), (350, 0), red, thickness=-1)
    cv.rectangle(image, (895, 45), (1105, 130), white, thickness=-1)
    cv.rectangle(image, (1015, 120), (1105, 380), white, thickness=-1)
    cv.rectangle(image, (895, 370), (1105, 455), white, thickness=-1)
    cv.rectangle(image, (900,50),(1100,125),yellow,thickness=-1)
    cv.rectangle(image, (1020, 125), (1100, 375), yellow, thickness=-1)
    cv.rectangle(image,(900,375),(1100,450),yellow,thickness=-1)
    cv.fillPoly(image,[vertices_clr],white)
    cv.fillPoly(image,[vertices],green)
    width = int(image.shape[1]*resize)
    height = int(image.shape[0]*resize)

    resized_image = cv.resize(image,(width,height))

    return resized_image

# Check if the node is within boundaries
def within_boundary(node):
    if 0 <= node[0] < int(500*resize) and 0 <= node[1] < int(1200*resize):
        return True

# Check if the node is in free-space
def in_free_space(image,node):
    if np.all(image[node[0],node[1]] == (0,0,0)):
        return True

# Move the robot up
def up(image, node):
    new_node = (node[0] + 1, node[1])
    if within_boundary(new_node) and in_free_space(image,new_node):
        return (new_node, 1)

# Move the robot down
def down(image, node):
    new_node = (node[0] - 1, node[1])
    if within_boundary(new_node) and in_free_space(image,new_node):
        return (new_node, 1)

# Move the robot to the left
def left(image, node):
    new_node = (node[0], node[1] - 1)
    if within_boundary(new_node) and in_free_space(image,new_node):
        return (new_node, 1)

# Move the robot to the right
def right(image, node):
    new_node = (node[0], node[1] + 1)
    if within_boundary(new_node) and in_free_space(image,new_node):
        return (new_node, 1)


# Move the robot up diagonally to the right
def up_right(image, node):
    new_node = (node[0] + 1, node[1] + 1)
    if within_boundary(new_node) and in_free_space(image,new_node):
        return (new_node, np.sqrt(2))

# Move the robot up diagonally to the left
def up_left(image, node):
    new_node = (node[0] + 1, node[1] - 1)
    if within_boundary(new_node) and in_free_space(image,new_node):
        return (new_node, np.sqrt(2))

# Move the robot down diagonally to the right
def down_right(image, node):
    new_node = (node[0] - 1, node[1] + 1)
    if within_boundary(new_node) and in_free_space(image,new_node):
        return (new_node, np.sqrt(2))

# Move the robot down diagonally to the left
def down_left(image, node):
    new_node = (node[0] - 1, node[1] - 1)
    if within_boundary(new_node) and in_free_space(image,new_node):
        return (new_node, np.sqrt(2))

# Grouping all the actions in a list
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

# writing backtrack function
def backtrack(path,start_node,goal_node):
    node = goal_node
    final_path = []
    while node is not start_node:
        final_path.append(node)
        node = path[node]
    final_path.append(start_node)
    final_path.reverse()
    return final_path

# performing dijkstra algorithm
def dijkstra(image,start_node,goal_node):
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
    return final_path, list(closed_list.keys())

# The main script
start_time = time.time()
image = map()
image_out = [image]
shortest_path = []
travelled_node = []
while True:
    start, goal = user_input()
    if in_free_space(image,start) and in_free_space(image,goal):
        path,all_nodes = dijkstra(image,start,goal)
        shortest_path = path
        travelled_node = all_nodes
        break
    else:
        print("In obstacle space, reenter new coordinates")

# Processing video frames for animation
for i in range(len(travelled_node)):
    img_copy = np.copy(image_out[i])
    img_copy[travelled_node[i][0],travelled_node[i][1]] = (76,153,0)
    image_out.append(img_copy)

for j in range(len(shortest_path)):
    img_copy = np.copy(image_out[-1])
    img_copy[shortest_path[j][0],shortest_path[j][1]] = (204,204,204)
    image_out.append(img_copy)

# Creating the video
frame_width = image.shape[1]
frame_height = image.shape[0]
fps = 15
fourcc = cv.VideoWriter_fourcc(*'mp4v')
out = cv.VideoWriter('output_video.mp4', fourcc, fps, (frame_width, frame_height))

counter = 0
for i in range(len(image_out)):
    counter += 1
    if counter%10 != 0:
        continue
    else:
        out.write(cv.flip(image_out[i],0))
    

out.release()

end_time = time.time()
print(end_time-start_time)

