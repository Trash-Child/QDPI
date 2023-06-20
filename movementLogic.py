from Camera.BallAnalysis import analyseFrame, locate_nearest_ball, findRobot
from ObstacleAnalysis import detectX
import cv2
import math
import numpy as np


# CONSTANTS
safetydistance = 50

# This function extracts important information from a frame captured by a camera.
# It analyzes the frame to locate white balls, orange balls, green balls, and blue balls.
# It then finds the nearest white ball and the nearest robot ball.
# If any of the required information is missing, it returns None.
def getImportantInfo(frame, debugFrame):
    _, _, white_balls, orange = analyseFrame(frame, debugFrame)
    robot, heading = findRobot(frame, debugFrame)
    target = locate_nearest_ball(white_balls, orange, robot)

    # Extract the coordinates of the target ball, robot ball, and green ball.
    target = target[0][:2]
    try:
        target = int(target[0]), int(target[1])
    except Exception as e:
        print("No target")
    robot = int(robot[0]), int(robot[1])
    return target, robot, heading

def distanceToBall(frame, debugframe):
    target_pos, robot_pos, _ = getImportantInfo(frame, debugframe)
    dist = getDistance(target_pos, robot_pos)
    if dist > 150:
        dist = dist - 100
    elif dist <= 150:
        dist = 200
    return dist

def getDistance(target, robot):
    target_coords = np.array(target)
    robot_coords = np.array(robot)
    difference = robot_coords - target_coords
    distance = np.sqrt(np.sum(np.square(difference)))
    return distance

# This function calculates the angle between three points.
def get_desired_heading(target, robot_pos, robot_heading):
    if target is None or robot_pos is None:
        return None

    direction_vector = np.array([target[0] - robot_pos[0], target[1] - robot_pos[1]])
    desired_heading = (np.arctan2(direction_vector[1], direction_vector[0]) * 180 / np.pi +180) % 360
    
    #the new stuff 
    if desired_heading <= 25 and desired_heading >= 335:
        desired_heading = desired_heading - 18
    
    elif desired_heading <= 155 and desired_heading <= 205: 
        desired_heading = desired_heading + 8
    #end of the new stuff 
    
    relative_angle = (desired_heading - robot_heading - 90) % 360 #tallet som er 108 nu var engang 90. Godt at huske som base. 
    return relative_angle 


def setupDelivery(robot, heading):
    if heading <= 265 or heading >= 275:
        angleRobotNeedsToTurnToFaceSouthWhichIs90
        return (90 - heading) % 360 # turn
    else:
        return 2 # open gate
  
def calculateCommandToGoal(frame, debugFrame, isHeadedStraight):
    mid_w, mid_e, _, _ = analyseFrame(frame, debugFrame)
    robot, heading = findRobot(frame, debugFrame)
    if robot is None:
        return 404
    angle = get_desired_heading(mid_e, robot, heading) # Change is west is big goal
    dist = getDistance(robot, mid_e) # Change if west is big goal
    if dist < 150 and not isHeadedStraight:
        return setupDelivery(robot, heading)

    elif abs(angle) >= 5:
        return angle
    
    elif isHeadedStraight:
        return dist-100

    else:
        return 1

# This function calculates the command based on the given frame.
# It calls the getImportantInfo function to extract the necessary information.
# If the robot ball is missing, it returns 404.
# Otherwise, it calculates the angle between the green ball, robot ball, and target ball.
# If the angle is greater than 5 degrees, it returns the angle.
# Otherwise, it returns 1.
def calculateCommand(frame, debugFrame):
    target, robot, heading = getImportantInfo(frame, debugFrame)
    if robot is None:
        return 404
    
    angle = get_desired_heading(target, robot, heading)
	

    if abs(angle) > 5 and angle < 355: # turn if above 5 degrees
        return angle
    else:
        return 1 # go straight

# Function to check if two lines intersect
def lineIntersection(line1, line2):
    # Line1 and line2 are 2D vectors with the format ((x1, y1), (x2, y2))

    # Calculate the difference in x and y coordinates for each line
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    # Calculate the determinant of xdiff and ydiff to check if lines are parallel
    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       return False

    # Calculate x and y coordinates of the intersection point
    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    # Return the intersection point
    return x, y  

# Function to check if path to bal intersects with any obstacle lines and call avoidObstacle if intersection
def intersectionLogic(robot, target, line_vectors, course_limits):
    # The path from the robot to the target is initially a straight line
    path = (robot, target)

    # Check if the path intersects with any of the obstacles
    for obstacleLine in line_vectors:
        intersection_point = lineIntersection(path, obstacleLine)

        # If there is an intersection point, we need to adjust our path
        if intersection_point is not None and intersection_point is not False:
            avoidObstacle(course_limits)
    return #return statement here

 
# Begin:    Dictes avoidObstacle
def getRobotSide(x_limits): # x_limits is list [lo_x, hi_x, lo_y, hi_y]. Returns side as char (w,e,n,s) 
	robotXY, robotHeading = findRobot(frame, debugFrame) 
	
	side = 'x'
	safe = 2

	# West of X
	if robotXY[0] < x_limits[0]:						
		side = 'w'										# QDPI west of X
		if robotXY[0] < (x_limits[0]-safetydistance):	
			safe = 1									# QDPI is clear to go around
		else: 											# QDPI is too close to X, get clear
			safe = 0

	# East of X
	elif robotXY[0] > x_limits[1]:						
		side = 'e'										
		if robotXY[0] > (x_limits[1]+safetydistance): 	# QDPI is clear
			safe = 1
		else: 											# QDPI is too close to X, get clear
			safe = 0
	
	# North of X
	elif robotXY[1] < x_limits[2]:
		side = 'n'
		if robotXY[1] < (x_limits[2]-safetydistance): 	# QDPI is clear
			safe = 1
		else:
			safe = 0

	# South of X
	elif robotXY[1] > x_limits[3]:
		side = 's'
		if robotXY[1] > (x_limits[3]+safetydistance):
			safe = 1
		else:
			safe = 0

	#Something went wrong
	else: 
		print("Side and safe not set! \n")

	return side, safe

def getSafe(side):
	robotXY, robotHeading = findRobot(frame, debugFrame) 
	if side == 'w':
		if robotHeading < 90 or robotHeading > 270: #facing X
			return -1 	# reverse 100
		else:
			return 3

	elif side == 'e':
		if robotHeading > 90 and robotHeading < 270: #facing X
			return -1 	# reverse 100
		else:
			return 3
	
	elif side == 'n': 		#facing X						
		if robotHeading > 180:
			return -1
		else:
			return 3

	elif side == 's':
		if robotHeading < 180: #facing X
			return -1
		else:
			return 3
	
	else: 
		print("error in getSafe!\n")

def avoidObstacle(course_limits):
	x_limits, running_avg, lineVectors = detectX(frame, debugFrame, running_avg, alpha=0.2) #lineVectors is not used in this
	robotXY, robotHeading = findRobot(frame, debugFrame)
	side, safe = getRobotSide(x_limits) #safe is int (0,1 are valid values)
	if safe == 2:
		print("Error with safe in avoidObstacle")
	else:
		if safe == 0:
			return getSafe(side)
	
	if side == 'e':
		if robotHeading < 85 or robotHeading > 95:	# Heading is not north
			desired_heading = 90
			relative_angle = (desired_heading - robotHeading + 180) % 360 - 180
			return relative_angle
		else:
			manualDistance = robotXY[1]-(course_limits[2]+safetydistance) #drive north
			return 3, manualDistance
	
	elif side == 'n':
		if robotHeading < 175 or robotheading > 185: # Heading is not west
			desired_heading = 180
			relative_angle = (desired_heading - robotHeading + 180) % 360 - 180
			return relative_angle
		else:
			manualDistance = robotXY[0]-(course_limits[0]+safetydistance) #drive west
			return 3, manualDistance
	
	elif side == 'w':
		if robotHeading < 265 or robotHeading > 275: # heading is not south
			desired_heading = 270
			relative_angle = (desired_heading - robotHeading + 180) % 360 - 180
			return relative_angle
		else:
			manualDistance = (course_limits[3]-safetydistance)-robotXY[1] #drive south
			return 3, manualDistance
	
	elif side == 's':
		if robotHeading > 5 and robotHeading < 355: # heading is not east
			desired_heading = 0
			relative_angle = (desired_heading - robotHeading + 180) % 360 - 180
			return relative_angle
		else:
			manualDistance = (course_limits[1]-safetydistance)-robotXY[0] #drive east
			return 3, manualDistance
	
	else:
		print("error with side in avoidObstacle!\n")
# End:      Dictes avoidObstacle

