from Camera.BallAnalysis import analyseFrame, locate_nearest_ball
import cv2
import math

# This function extracts important information from a frame captured by a camera.
# It analyzes the frame to locate white balls, orange balls, green balls, and blue balls.
# It then finds the nearest white ball and the nearest robot ball.
# If any of the required information is missing, it returns None.
def getImportantInfo(frame):
    white_balls, orange, green, blue = analyseFrame(frame)
    target, robot = locate_nearest_ball(white_balls, orange, green, blue)
    if target is None or robot is None or green is None:
        print("getImportantInfo returning none")
        return None, None, None

    # Extract the coordinates of the target ball, robot ball, and green ball.
    target = target[0][:2]
    robot = int(robot[0]), int(robot[1])
    green = int(green[0]), int(green[1])

    # Draw a circle around the target ball on the frame.
    cv2.circle(frame, target, 10, (255, 255, 0), 2)

    return target, robot, green   

# This function calculates the angle between three points.
def calculateAngle(p1, p2, p3):
    dx21 = p2[0] - p1[0]
    dy21 = p2[1] - p1[1]
    dx32 = p3[0] - p2[0]
    dy32 = p3[1] - p2[1]
    angle = math.atan2(dy32, dx32) - math.atan2(dy21, dx21)
    return round(math.degrees(angle))

# This function calculates the command based on the given frame.
# It calls the getImportantInfo function to extract the necessary information.
# If the robot ball is missing, it returns 404.
# Otherwise, it calculates the angle between the green ball, robot ball, and target ball.
# If the angle is greater than 5 degrees, it returns the angle.
# Otherwise, it returns 1.
def calculateCommand(frame):
    target, robot, green = getImportantInfo(frame)
    if robot is None:
        return 404
    
    angle = calculateAngle(green, robot, target)
    if abs(angle) > 5: # error margin
        return angle
    else:
        return 1
