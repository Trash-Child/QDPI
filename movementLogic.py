from Camera.BallAnalysis import analyseFrame, locate_nearest_ball, findRobot
import cv2
import math
import numpy as np

NW = (46,45) # SET MANUALLY
SE = (560,428) # SET MANUALLY

# This function extracts important information from a frame captured by a camera.
# It analyzes the frame to locate white balls, orange balls, green balls, and blue balls.
# It then finds the nearest white ball and the nearest robot ball.
# If any of the required information is missing, it returns None.
# Anton 100%
def getImportantInfo(frame, debugFrame):
    _, _, white_balls, orange = analyseFrame(frame, debugFrame, False)
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

# Anton 50%, Maria 50%
def distanceToBall(frame, debugframe):
    target_pos, robot_pos, heading = getImportantInfo(frame, debugframe)
    dist = getDistance(target_pos, robot_pos, heading)
    if dist > 150:
        dist = dist - 100
    elif dist <= 150:
        dist = 200
    return dist

# Anton 45%, Maria 45%, Benedicte 10%
def getDistance(target, robot, heading):
    target_coords = np.array(target)
    robot_coords = np.array(robot)
    target_coords = checkDistCollision(target_coords, NW, SE) #check target coords are within bounds
    difference = robot_coords - target_coords
    distance = np.sqrt(np.sum(np.square(difference)))
    return distance

# This function calculates the angle between three points.
# Anton 50%, Maria 50%
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
    
    relative_angle = (desired_heading - robot_heading + 180) % 360 - 180 #tallet som er 108 nu var engang 90. Godt at huske som base. 

    return relative_angle - 90

# Anton 50%, Maria 50%
def setupDelivery(robot, heading):

    angleToSouth = - (270 - heading) % 360 # 90 = north. Might need to be changed in case of diff number
    print('Heading: ', heading)
    print('angleToSouth: ', angleToSouth)
    if angleToSouth < 5 or angleToSouth > 355:
        return 2
    else:
        return angleToSouth

# Anton 50%, Maria 50%
def calculateCommandToGoal(frame, debugFrame, isHeadedStraight):
    mid_w, mid_e, _, _ = analyseFrame(frame, debugFrame, False)
    print(mid_e)
    robot, heading = findRobot(frame, debugFrame)
    if robot is None:
        return 404
    angle = get_desired_heading(mid_w, robot, heading) # Change is west is big goal
    print("angle:", angle)
    dist = getDistance(robot, mid_w, heading)# Change if west is big goal
    print("Dist:", dist)
    if dist < 150:
        if dist < 40:
            return -1
        return setupDelivery(robot, angle)

    elif abs(angle) >= 5 and abs(angle) <= 355:
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
# Anton 50%, Maria 50%
def calculateCommand(frame, debugFrame):
    target, robot, heading = getImportantInfo(frame, debugFrame)
    if robot is None:
        return 404
    
    angle = get_desired_heading(target, robot, heading)
    if abs(angle) > 5 and abs(angle) < 355: # turn if above 5 degrees
        return angle
    else:
        return 1 # go straight

# Benedicte 100%
def checkDistCollision(target_coords, NW, SE):
    safetyDistance = 40
    targetX = target_coords[0]
    targetY = target_coords[1]

    if targetX < NW[0]+safetyDistance: # target is too far west
	    diff = (NW[0]+safetyDistance) - targetX
	    targetX = targetX + diff
	
    elif targetX > SE[0]-safetyDistance: # target is too far east
        diff = targetX - (SE[0]-safetyDistance)
        targetX = targetX - diff
    
    if targetY < NW[1]+safetyDistance: # target is too far north
        diff = (NW[1]+safetyDistance) - targetY
        targetY = targetY + diff
	
    elif targetY > SE[1]+safetyDistance: # target is too far south
        diff = targetY - (SE[1]+safetyDistance)
        targetY = targetY - diff
	
	# newDist = np.sqrt(targetX**2 + targetY**2) # If target is within bounds, distance remains the same
    return np.array([targetX,targetY])









