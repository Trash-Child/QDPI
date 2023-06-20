# Import necessary libraries and modules
from Camera.BallAnalysis import analyseFrame, locate_nearest_ball, findRobot
from Camera.ObstacleAnalysis import detectX
from client import executeCommand
import cv2
import math
import numpy as np

# Function to extract important information from a frame captured by a camera.
def getImportantInfo(frame, debugFrame):
    try:
        # Analyze the frame to locate white balls, orange balls, green balls, and blue balls.
        # Find the nearest white ball and the nearest robot ball.
        white_balls, orange = analyseFrame(frame, debugFrame)
        robot, heading = findRobot(frame, debugFrame)
        target = locate_nearest_ball(white_balls, orange, robot)
    except Exception as e:
        print(f"Error occurred while analysing frame: {e}")
        return None, None, None  # If there's an error, return None values

    # Process the target coordinates
    if target is not None:
        try:
            target = target[0][:2]
            target = int(target[0]), int(target[1])
        except Exception as e:
            print(f"Error occurred while processing target: {e}")
            target = None
    else:
        print("No target found.")
        target = None

    # Process the robot coordinates
    if robot is not None:
        try:
            robot = int(robot[0]), int(robot[1])
        except Exception as e:
            print(f"Error occurred while processing robot: {e}")
            robot = None

    # Return the coordinates of target, robot, and heading
    return target, robot, heading


def calculate_distance(frame, debugframe):
    target_pos, robot_pos, _ = getImportantInfo(frame, debugframe)
    robot_coords = np.array(robot_pos)
    target_coords = np.array(target_pos)
    difference = robot_coords - target_coords
    # Pythagorean theorem
    distance = np.sqrt(np.sum(np.square(difference)))

    return distance    


# Function to calculate the angle between three points
def get_heading_to_ball(ball, robot_pos, robot_heading):
    # If either the ball or robot's position is None, return None
    if ball is None or robot_pos is None:
        return None

    # Compute the direction vector from the robot to the ball
    direction_vector = np.array([ball[0] - robot_pos[0], ball[1] - robot_pos[1]])
    # Compute the desired heading in degrees and make sure it is within [0, 360)
    desired_heading = (np.arctan2(direction_vector[1], direction_vector[0]) * 180 / np.pi +180) % 360
    # Compute the relative angle to the current heading of the robot, and make sure it is within [-180, 180)
    relative_angle = (desired_heading - robot_heading + 180) % 360 - 180

    return relative_angle-85

intersections = False  # Global boolean if obstacle in the way

def calculateCommand(frame, debugFrame):
    global intersections  # Declare intersections as global

    # Extract important information from the frame
    try:
        target, robot, robot_heading = getImportantInfo(frame, debugFrame)
        print(robot, robot_heading)
    except Exception as e:
        print(f"Error when calling getImportantInfo: {e}")
        return None

    # Check if robot or target or robot_heading is None, if so, return None
    if robot is None or target is None or robot_heading is None:
        print("Skipping iteration due to missing data.")
        return None

    # Get the obstacle information from the frame
    try:
        lineVectors = getObstacleInfo(frame, debugFrame)
    except Exception as e:
        print(f"Error when calling getObstacleInfo: {e}")
        return None

    # If obstacle info is None, return None
    if lineVectors is None:
        print("Skipping iteration due to missing obstacle information.")
        return None

    # If there are no intersections, go to ball
    if intersections == False:
        try:
            intersections = avoidObstacles(frame, debugFrame, robot, target, line_vectors, heading)
            if intersections == True:
                target, robot, heading = getImportantInfo(frame, debugFrame)
                if robot is None:
                    return 404
                
                angle = get_heading_to_ball(target, robot, heading)
                if abs(angle) > 5: # turn if above 5 degrees
                    return angle
                else:
                    return 1 # go straight

        except Exception as e:
            print(f"Error when moving to ball without obstacle {e}")
            return None

# Function to extract obstacle information from the frame
def getObstacleInfo(frame, debugFrame):
    # Get line vectors of detected obstacles
    lineVectors = detectX(frame, debugFrame)
    if lineVectors is None:
        print("getObstacleInfo returning none")
        return None
    # Return line vector array
    return lineVectors

# Function to check if path to ball is intersected by the obstacle
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

def turnNorth(heading):
    # The desired heading is 90 degrees
    desired_heading = 90

    # Compute the relative angle to the current heading of the robot, and make sure it is within [-180, 180)
    relative_angle = (desired_heading - heading + 180) % 360 - 180
    
    # Return the relative angle
    return relative_angle    

def avoidObstacles(frame, debugFrame, robot, target, line_vectors, heading):
    # The path from the robot to the target is initially a straight line
    path = (robot, target)

    global intersections  # Declare intersections as global

    commands = []  # List to store the commands

    # Check if the path intersects with any of the obstacles
    for obstacleLine in line_vectors:
        intersection_point = lineIntersection(path, obstacleLine)

        # If there is an intersection point, we need to adjust our path
        if intersection_point is not None and intersection_point is not False:
            intersections = True
            # Turn towards top of frame
            turn_command = turnNorth(heading)
            commands.append(turn_command)
            
            # Determine the direction of the obstacle relative to the robot
            if intersection_point[0] < robot[0]:  # Obstacle is to the left
                # Move forward towards top frame
                move_command = 1
                commands.append(move_command)

                # Turn left
                turn_command = 180
                commands.append(turn_command)

                # Move forward twice to complete the half square path
                move_command = 1
                commands.append(move_command)
                commands.append(move_command)
            else: # Obstacle is to the right
                # Move forward to avoid obstacle
                move_command = 1
                commands.append(move_command)

                # Turn right
                turn_command = 0
                commands.append(turn_command)

                # Move forward to complete the half square path
                move_command = 1
                commands.append(move_command)
                commands.append(move_command)
    return commands

                