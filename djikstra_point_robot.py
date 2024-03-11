# importing important libraries
import cv2 as cv
import numpy as np


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

image = map()
cv.imshow("map",image)
cv.waitKey(0)

