from Camera.BallAnalysis import analyseFrame, locate_nearest_ball, findRobot
import cv2
import math
import numpy as np

# This function extracts important information from a frame captured by a camera.
# It analyzes the frame to locate white balls, orange balls, green balls, and blue balls.
# It then finds the nearest white ball and the nearest robot ball.
# If any of the required information is missing, it returns None.
def getImportantInfo(frame, debugFrame):
    mid_w, mid_e, white_balls, orange = analyseFrame(frame, debugFrame)
    robot, heading = findRobot(frame, debugFrame)
    target = locate_nearest_ball(white_balls, orange, robot)
    # Extract the coordinates of the target ball, robot ball, and green ball.
    try:
        target = int(target[0]), int(target[1])
    except Exception as e:
        print("No target")
    robot = int(robot[0]), int(robot[1])
    return target, robot, heading

def calculate_distance(robot, target):
    target = target[0][:2]
    difference = robot - target
    # Pythagorean theorem
    distance = np.sqrt(np.sum(np.square(difference)))

    return distance

# This function calculates the angle between three points.
def get_heading_to_ball(ball, robot_pos, robot_heading):
    if ball is None or robot_pos is None:
        return None

    direction_vector = np.array([ball[0] - robot_pos[0], ball[1] - robot_pos[1]])
    desired_heading = (np.arctan2(direction_vector[1], direction_vector[0]) * 180 / np.pi +180) % 360
    relative_angle = (desired_heading - robot_heading + 180) % 360 - 180

    return relative_angle-85

def get_heading_to_goal(goal, robot_pos, robot_heading):
    if goal or robot_pos is None:
        return None

    direction_vector = np.array([goal[0] - robot_pos[0], goal[1] - robot_pos[1]])
    desired_heading = (np.arctan2(direction_vector[1], direction_vector[0]) * 180 / np.pi + 180) % 360
    relative_angle = (desired_heading - robot_heading + 180) % 360 - 180

    return relative_angle-85 

def setupDelivery(angleToGoal, robot, heading):
    adjusted_angle_to_goal = (angleToGoal - 90) % 360 # turning -90 to have the right side of the robot face the goal. OBS Might need to be changed to 90
    if abs(adjusted_angle_to_goal) >= 5:
        return abs(adjusted_angle_to_goal) # turn
    else:
        return 2 # open gate
    

def calculateCommandToGoal(frame, debugFrame):
    mid_w, mid_e, _, _ = analyseFrame(frame, debugFrame, noBalls)
    robot, heading = findRobot(frame, debugFrame)
    if robot is None:
        return 404
    angle = get_heading_to_goal(mid_e, robot, heading) # Change is west is big goal
    dist = calculate_distance(robot, mid_e) # Change if west is big goal
    if dist < 50:
        return setupDelivery(angle, robot, heading)
    elif abs(angle) >= 5:
        return angle
    else:
        return 1, dist


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
    
    angle = get_heading_to_ball(target, robot, heading)
    if abs(angle) > 5: # turn if above 5 degrees
        return angle, None
    else:
        return 1, calculate_distance(robot, target) # go straight
