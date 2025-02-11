# Dijkstras Algortihm for path planning of a point robot in 2D space
**Note:** Note: This project was a requirement for the course ENPM 661- Planning for Autonomous Robots at University of Maryland, College Park

## Project Description:
This script contains an implementation of the Dijkstras algorithm for a point robot for navigation around obstacles in a 2D space.

## Dependencies:
* python 3.11 (any version above 3 should work)
* Python running IDE (We used Pycharm)
  
## Libraries Used:
* OpenCV
* Numpy
* HeapQ
* Time

## Instructions 
1. Download the zip file and extract it
	
2. Install python and the required dependencies: 

	`pip install numpy opencv-python`
	
3. Run the code or use desired python IDE:

	`$python3 djikstra_point_robot.py`

Input the X and Y coordinates of the start and goal node, when prompted in the Terminal

## Demo
![rrtconnect_apf](https://github.com/user-attachments/assets/f12cd0e4-c706-4bc8-8f5b-5019c0dca440)


**Note:** The resolution of the output video is low because the array generated (image) is of high dimensions. You coud resize the output  dimensions, by changing the 'resize' variable factor at the top of the code. If in case you get numpy exception error, where in memory is not enough to store the array, check this link:
https://stackoverflow.com/questions/57507832/unable-to-allocate-array-with-shape-and-data-type

