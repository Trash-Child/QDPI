from Camera.BallAnalysis import analyseFrame, locate_nearest_ball, findRobot
import cv2
from Camera.detectx import detectX
import math
import numpy as np

# This function extracts important information from a frame captured by a camera.
# It analyzes the frame to locate white balls, orange balls, green balls, and blue balls.
# It then finds the nearest white ball and the nearest robot ball.
# If any of the required information is missing, it returns None.
def getImportantInfo(frame, debugFrame):
    white_balls, orange = analyseFrame(frame, debugFrame)
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

def calculate_distance(frame, debugframe):
    target_pos, robot_pos, _ = getImportantInfo(frame, debugframe)
    robot_coords = np.array(robot_pos)
    target_coords = np.array(target_pos)
    difference = robot_coords - target_coords
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
    
    # Get the obstacle information from the frame
    try:
        obstacle_info = getObstacleInfo(frame, debugFrame)
    except Exception as e:
        print(f"Error when calling getObstacleInfo: {e}")
        return None


    angle = get_heading_to_ball(target, robot, heading)
    if abs(angle) > 5: # turn if above 5 degrees
        return angle
    else:
        return 1 # go straight


# Function to extract obstacle information from the frame
def getObstacleInfo(frame, debugFrame):
    # Get line vectors of detected obstacles
    lineVectors = detectX(frame, debugFrame)
    if lineVectors is None:
        print("getObstacleInfo returning none")
        return None
    # Return line vector array
    return lineVectors