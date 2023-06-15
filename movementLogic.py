from Camera.BallAnalysis import analyseFrame, locate_nearest_ball, findRobot
import cv2
import math

# This function extracts important information from a frame captured by a camera.
# It analyzes the frame to locate white balls, orange balls, green balls, and blue balls.
# It then finds the nearest white ball and the nearest robot ball.
# If any of the required information is missing, it returns None.
def getImportantInfo(frame):
    white_balls, orange = analyseFrame(frame)
    robot, heading = findRobot(frame)
    target = locate_nearest_ball(white_balls, orange, robot)

    # Extract the coordinates of the target ball, robot ball, and green ball.
    target = target[0][:2]
    # make sure it's ints
    robot = int(robot[0]), int(robot[1])
    cv2.circle(frame, target, 10, (255, 255, 0), 2)

    return target, robot, heading   

# This function calculates the angle between three points.
def get_heading_to_ball(ball, robot_pos, robot_heading):
    if ball is None or robot_pos is None:
        return None

    direction_vector = np.array([ball[0] - robot_pos[0], ball[1] - robot_pos[1]])
    desired_heading = (np.arctan2(direction_vector[1], direction_vector[0]) * 180 / np.pi + 360) % 360
    relative_angle = (desired_heading - robot_heading + 180) % 360 - 180

    return relative_angle

# This function calculates the command based on the given frame.
# It calls the getImportantInfo function to extract the necessary information.
# If the robot ball is missing, it returns 404.
# Otherwise, it calculates the angle between the green ball, robot ball, and target ball.
# If the angle is greater than 5 degrees, it returns the angle.
# Otherwise, it returns 1.
def calculateCommand(frame):
    target, robot, heading = getImportantInfo(frame)
    if robot is None:
        return 404
    
    angle = get_heading_to_ball(target[0], robot, heading)
    if abs(angle) > 5: # turn if above 5 degrees
        return angle
    else:
        return 1 # go straight
