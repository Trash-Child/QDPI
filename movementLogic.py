# Import necessary libraries and modules
from Camera.BallAnalysis import analyseFrame, locate_nearest_ball, findRobot
from Camera.ObstacleAnalysis import detectX
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

waypoints = []  # Global list to hold the waypoints
turn_completed = False  # Global flag to indicate if a turn has been completed
threshold_distance = 10  # Threshold distance to consider a waypoint as reached

def calculateCommand(frame, debugFrame):
    global waypoints  # Declare waypoints as global

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
        obstacle_info = getObstacleInfo(frame, debugFrame)
    except Exception as e:
        print(f"Error when calling getObstacleInfo: {e}")
        return None

    # If obstacle info is None, return None
    if obstacle_info is None:
        print("Skipping iteration due to missing obstacle information.")
        return None

    # If there are no waypoints, calculate them
    if not waypoints:
        try:
            waypoints = avoidObstacles(frame, debugFrame, robot, target, obstacle_info, robot_heading)
        except Exception as e:
            print(f"Error when calling avoidObstacles: {e}")
            return None

    # Get the first waypoint and its heading
    waypoint, waypoint_heading = waypoints[0]

    # Compute the heading to the waypoint
    try:
        angle = get_heading_to_ball(waypoint, robot, robot_heading)
    except Exception as e:
        print(f"Error when calculating turning angle: {e}")
        return None

    # If the robot is close enough to the waypoint
    if math.hypot(waypoint[0] - robot[0], waypoint[1] - robot[1]) < 10:
        # And if within the threshold angle
        if abs(angle) <= 1:
            # Remove this waypoint from the list
            waypoints.pop(0)

    # If the absolute angle is above 5 degrees, turn towards waypoint/target
    try:
        if abs(angle) > 5:
            return angle
        else:  # Otherwise, go straight
            return 1
    except Exception as e:
        print(f"Error occurred during command decision: {e}")
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

def avoidObstacles(frame, debugFrame, robot, target, line_vectors, heading):
    # The path from the robot to the target is initially a straight line
    path = (robot, target)

    # Start with the robot's initial position and its current heading
    waypoints = [(robot, heading)]

    # Check if the path intersects with any of the obstacles
    for obstacleLine in line_vectors:
        intersection_point = lineIntersection(path, obstacleLine)

        # If there is an intersection point, we need to adjust our path
        if intersection_point is not None and intersection_point is not False:
            # Determine the direction of the obstacle relative to the robot
            if intersection_point[0] < robot[0]:  # Obstacle is to the left
                # Turn towards top of frame
                waypoint = (robot[0], 0)
                waypoint_heading = 90  # facing upward

                waypoints.append((waypoint, waypoint_heading))  # append the waypoint and heading

                # Move forward to avoid obstacle
                waypoint = (robot[0], 0 - 20)  # move forward by 20 units
                waypoint_heading = 90  # still facing upward
                
                waypoints.append((waypoint, waypoint_heading))  # append the waypoint and heading
                
                # Turn right
                waypoint_heading = 0  # facing right
                waypoints.append((waypoint, waypoint_heading))  # append the waypoint and heading

                # Move forward to go around the obstacle
                waypoint = (waypoint[0] + 20, waypoint[1])  # move right by 20 units
                
                waypoints.append((waypoint, waypoint_heading))  # append the waypoint and heading

            else:  # Obstacle is to the right
                # Turn towards top of frame
                waypoint = (robot[0], 0)
                waypoint_heading = 90  # facing upward

                waypoints.append((waypoint, waypoint_heading))  # append the waypoint and heading

                # Move forward to avoid obstacle
                waypoint = (robot[0], 0 - 20)  # move forward by 20 units
                waypoint_heading = 90  # still facing upward
                
                waypoints.append((waypoint, waypoint_heading))  # append the waypoint and heading
                
                # Turn left
                waypoint_heading = 180  # facing left
                waypoints.append((waypoint, waypoint_heading))  # append the waypoint and heading

                # Move forward to go around the obstacle
                waypoint = (waypoint[0] - 20, waypoint[1])  # move left by 20 units
                
                waypoints.append((waypoint, waypoint_heading))  # append the waypoint and heading

            # After avoiding the obstacle, robot should head towards the target
            path = (waypoint, target)
            heading = get_heading_to_ball(target, robot, waypoint_heading)

    # Add the target to the list of waypoints
    target_heading = get_heading_to_ball(target, robot, heading)
    waypoints.append((target, target_heading))

    return waypoints

