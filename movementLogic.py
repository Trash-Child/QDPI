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
    try:
        target, robot, heading = getImportantInfo(frame, debugFrame)
    except Exception as e:
        print(f"Error when calling getImportantInfo: {e}")
        return None

    if robot is None:
        return 404

    try:
        obstacle_info = getObstacleInfo(frame, debugFrame)
    except Exception as e:
        print(f"Error when calling getObstacleInfo: {e}")
        return None

    if target is None or obstacle_info is None:
        try:
            angle = get_heading_to_ball(target, robot, heading)
        except Exception as e:
            print(f"Error when calling get_heading_to_ball: {e}")
            return None
    else:
        try:
            waypoints = avoidObstacles(frame, debugFrame, robot, target, obstacle_info)
        except Exception as e:
            print(f"Error when calling avoidObstacles: {e}")
            return None

        if not waypoints or not isinstance(waypoints[0], (tuple, list)) or len(waypoints[0]) < 2:
            print(f"Expected waypoints[0] to be a tuple or a list with at least 2 elements, but got {waypoints[0] if waypoints else '[]'}.")
            try:
                angle = get_heading_to_ball(target, robot, heading)
            except Exception as e:
                print(f"Error when calling get_heading_to_ball: {e}")
                return None
        else:
            print(type(waypoints[0]), waypoints[0])  # Debug
            try:
                angle = get_heading_to_ball(waypoints[0], robot, heading)
            except Exception as e:
                print(f"Error when calling get_heading_to_ball: {e}")
                return None

            if math.hypot(waypoints[0][0] - robot[0], waypoints[0][1] - robot[1]) < 10:  # Threshold distance to waypoint
                waypoints.pop(0)

    try:
        if abs(angle) > 5:  # turn if above 5 degrees
            return angle
        else:
            return 1  # go straight
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


def avoidObstacles(frame, debugFrame, robot, target, line_vectors):
    # Path is a straight line from robot to target
    path = (robot, target)
    
    waypoints = [robot]  # Start with the robot's initial position

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
                    waypoint = (intersection_point[0] + 10, intersection_point[1])
                else:  # Obstacle is to the right
                    # Create a waypoint to the left of the obstacle
                    waypoint = (intersection_point[0] - 10, intersection_point[1])
                
                waypoints.append(waypoint)  # Add the waypoint to the list
                print(f"New waypoint added: {waypoint}")  # Debug

                # Update the robot's path to go from the waypoint to the target
                path = (waypoint, target)
            else:
                print(f"Unexpected intersection_point: {intersection_point}")

    # Add the target to the list of waypoints
    waypoints.append(target)
    
    print(f"Final waypoints: {waypoints}")  # Debug
    return waypoints

