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
    try:
        white_balls, orange = analyseFrame(frame, debugFrame)
        robot, heading = findRobot(frame, debugFrame)
        target = locate_nearest_ball(white_balls, orange, robot)
    except Exception as e:
        print(f"Error occurred while analysing frame: {e}")
        return None, None, None

    # Extract the coordinates of the target ball, robot ball, and green ball.
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

    if robot is not None:
        try:
            robot = int(robot[0]), int(robot[1])
        except Exception as e:
            print(f"Error occurred while processing robot: {e}")
            robot = None

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

waypoints = []  # Global variable to hold the waypoints
turn_completed = False
threshold_distance = 10  # Distance at which we consider the waypoint has been reached

def calculateCommand(frame, debugFrame):
    global waypoints  # Declare waypoints as a global variable

    try:
        target, robot, robot_heading = getImportantInfo(frame, debugFrame)
        print(robot, robot_heading)
    except Exception as e:
        print(f"Error when calling getImportantInfo: {e}")
        return None

    # Check if robot or target or robot_heading is None, if so, skip to the next iteration
    if robot is None or target is None or robot_heading is None:
        print("Skipping iteration due to missing data.")
        return None

    try:
        obstacle_info = getObstacleInfo(frame, debugFrame)
    except Exception as e:
        print(f"Error when calling getObstacleInfo: {e}")
        return None

    # Check if obstacle_info is None, if so, skip to the next iteration
    if obstacle_info is None:
        print("Skipping iteration due to missing obstacle information.")
        return None

    if not waypoints:
        try:
            waypoints = avoidObstacles(frame, debugFrame, robot, target, obstacle_info, robot_heading)
        except Exception as e:
            print(f"Error when calling avoidObstacles: {e}")
            return None

    waypoint, waypoint_heading = waypoints[0]
    try:
        angle = get_heading_to_ball(waypoint, robot, robot_heading)
    except Exception as e:
        print(f"Error when calculating turning angle: {e}")
        return None

    if math.hypot(waypoint[0] - robot[0], waypoint[1] - robot[1]) < 10:  # If close enough to waypoint
        if abs(angle) <= 1:  # And if within the threshold angle
            waypoints.pop(0)  # Remove this waypoint from the list

    try:
        if abs(angle) > 5:  # Turn if the absolute angle is above 5 degrees
            return angle  # Turn towards waypoint/target
        else:
            return 1  # Go straight
    except Exception as e:
        print(f"Error occurred during command decision: {e}")
        return None




# This function extracs the 2D vectors of the obstacle lines and stores them for further logic
# It calls detectX to get the line vectors
# It returns the line vector array
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
    # line1 and line2 are 2D vectors with the format ((x1, y1), (x2, y2))
    
    print(f"line1: {line1}, line2: {line2}")  # Debug

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


def avoidObstacles(frame, debugFrame, robot, target, line_vectors, heading):
    # Path is a straight line from robot to target
    path = (robot, target)

    waypoints = [(robot, heading)]  # Start with the robot's initial position and its current heading

    # Check if path intersects with any obstacle
    for obstacleLine in line_vectors:
        print(f"Checking obstacle line: {obstacleLine}")  # Debug

        intersection_point = lineIntersection(path, obstacleLine)
        print(f"Intersection point: {intersection_point}")  # Debug

        if intersection_point is not None and intersection_point is not False:
            # Check if intersection_point is a tuple or a list with at least 2 elements.
            if isinstance(intersection_point, (tuple, list)) and len(intersection_point) >= 2:
                # Determine the direction of the obstacle relative to the robot
                if intersection_point[0] < robot[0]:  # Obstacle is to the left
                    # Create a waypoint to the right of the obstacle
                    waypoint = (intersection_point[0] + 20, intersection_point[1] + 50)
                else:  # Obstacle is to the right
                    # Create a waypoint to the left of the obstacle
                    waypoint = (intersection_point[0] - 20, intersection_point[1] - 50)

                # Calculate heading to waypoint
                waypoint_heading = get_heading_to_ball(waypoint, robot, heading)

                waypoints.append((waypoint, waypoint_heading))  # Add the waypoint to the list along with its associated heading
                print(f"New waypoint added: {waypoint}")  # Debug

                # Update the robot's path to go from the waypoint to the target
                path = (waypoint, target)
                heading = waypoint_heading  # Update heading
            else:
                print(f"Unexpected intersection_point: {intersection_point}")

    # Add the target to the list of waypoints
    # Calculate heading to target
    target_heading = get_heading_to_ball(target, robot, heading)
    waypoints.append((target, target_heading))

    print(f"Final waypoints: {waypoints}")  # Debug
    return waypoints



