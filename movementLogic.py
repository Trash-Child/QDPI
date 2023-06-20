from Camera.BallAnalysis import analyseFrame, locate_nearest_ball, findRobot
import cv2
import math
import numpy as np

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
    if dist > 100:
        dist = dist - 150
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

''' this works enough, but trying to optimise 
    if robot_pos[1] > target[1]:
        relative_angle = relative_angle*1.1
    else:
        relative_angle = relative_angle*0.9
    return relative_angle 
'''


def setupDelivery(robot, heading):
    angleToSouth = - (270 - heading) % 360 # 90 = north. Might need to be changed in case of diff number
    if abs(heading) <= 265 or abs(heading) >= 275:
        return angleToSouth # turn
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
        return setupDelivery(robot, angle)

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
