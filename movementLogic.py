from Camera.BallAnalysis import analyseFrame, locate_nearest_ball
import cv2
import math


def getImportantInfo(frame):
    white_balls, orange, green, blue = analyseFrame(frame)
    target, robot = locate_nearest_ball(white_balls, orange, green, blue)

    print("target: ", target)
    if target is None or robot is None or green is None:
        print("target len", len(target))
        print("Returning none")
        return None, None, None
    target = target[0][:2]
    robot = int(robot[0]), int(robot[1])
    green = int(green[0]), int(green[1])
    # cv2.circle(frame, target, 10, (255, 255, 0), 2)
    return target, robot, green   
    

def calculateAngle(p1, p2, p3):
    print("calcAngle debug")
    dx21 = p2[0] - p1[0]
    dy21 = p2[1] - p1[1]
    dx32 = p3[0] - p2[0]
    dy32 = p3[1] - p2[1]
    angle = math.atan2(dy32, dx32) - math.atan2(dy21, dx21)
    print(round(math.degrees(angle)))
    return round(math.degrees(angle))

def calculateCommand(frame):
    target, robot, green = getImportantInfo(frame)
    if target is None or robot is None or green is None:
        return 404
    
    angle = calculateAngle(green, robot, target)
    print("Printing angle: ", angle)
    if abs(angle) > 5: # error margin
        return angle
        
    else:
        return 1

