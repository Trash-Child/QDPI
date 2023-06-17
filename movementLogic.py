from Camera.BallAnalysis import analyseFrame, locate_nearest_ball, findRobot
from Camera.ObstacleAnalysis import detectX
import cv2
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
    
    angle = get_heading_to_ball(target, robot, heading)
    if abs(angle) > 5: # turn if above 5 degrees
        return angle
    else:
        return 1 # go straight


# This function extracs the 2D vectors of the obstacle lines and stores them for further logic
# It calls detectX to get the line vectors
# It returns the line vector array
def getObstacleInfo(frame)
# Get line vectors of detected obstacles
lineVectors = detectX(frame)
if lineVectors = None:
    print("getObstacleInfo returning none")
    return None
# Return line vector array
return lineVectors

# Function to check if path to ball is intersected by the obstacle
def lineIntersection(line1, line2):
    # line1 and line2 are 2D vectors with the format ((x1, y1), (x2, y2))

    # Calculate the difference between the x-coordinates and the y-coordinates
    # for each line.
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    # The function 'det' calculates the determinant of a 2x2 matrix formed by
    # the two points on the line. This is a measure of the "signed" area parallelogram
    # spanned by these points. In this context, it's used to help find the
    # intersection point of the two lines.
    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    # Calculate the determinant of xdiff and ydiff. This checks if the two lines are parallel.
    # If they are, the determinant will be zero and we return False as there is no intersection point.
    div = det(xdiff, ydiff)
    if div == 0:
       return False

    # Compute the determinants of the two lines. The determinant is a special number that can be calculated
    # from a matrix. In this case, the matrix is a 2x2 matrix formed by the coordinates of the line's points.
    d = (det(*line1), det(*line2))

    # Calculate the x and y coordinates of the intersection point.
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    # Return the intersection point.
    return x, y


def avoidObstacles(frame, robot, target, line_vectors):
    # Path is a straight line from robot to target
    path = (robot, target)
    
    # Loop until a clear path is found
    while True:
        # Assume path is clear at the beginning of each iteration
        path_is_clear = True
        
        # Check if path intersects with any obstacle
        for obstacleLine in line_vectors:
            intersection_point = lineIntersection(path, obstacleLine)
            if intersection_point:
                path_is_clear = False  # Set path_is_clear to False if an obstacle is found
                
                # Determine the direction of the obstacle relative to the robot
                if intersection_point[0] < robot[0]:  # Obstacle is to the left
                    # Move the robot slightly to the right by increasing the x-coordinate
                    # And keeping the y-coordinate same
                    robot = (robot[0] + 10, robot[1])
                else:  # Obstacle is to the right
                    # Move the robot slightly to the left by decreasing the x-coordinate
                    # And keeping the y-coordinate same
                    robot = (robot[0] - 10, robot[1])
                
                # Update the robot's path
                path = (robot, target)
                break  # Exit the inner for-loop to start checking the new path
                
        # If path is clear, exit the loop
        if path_is_clear:
            break

    # At this point, a clear path should be found, and we can proceed to the target.
    # The function returns the new robot position so it can be used in other parts of your program.
    return robot


