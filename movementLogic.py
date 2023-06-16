from Camera.BallAnalysis import analyseFrame, locate_nearest_ball
from Camera.ObstacleAnalysis import detectX
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


