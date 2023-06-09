from Camera.BallAnalysis import runCamera, analyseFrame, locate_nearest_ball
import cv2
import math


def getImportantInfo(frame):
    white_balls, orange, green, blue = analyseFrame(frame)
    target, robot = locate_nearest_ball(white_balls, orange, green, blue)

    if target is not None and len(target) > 1:
        x, y = target[:2]
        cv2.circle(frame, (x, y), 10, (255, 255, 0), 2)
        return target, robot, green

def calculateAngle(p1, p2, p3):
    dx21 = p2[0] - p1[0]
    dy21 = p2[1] - p1[1]
    dx32 = p3[0] - p2[0]
    dy32 = p3[1] - p2[1]
    angle = math.atan2(dy32, dx32) - math.atan2(dy21, dx21)
    return math.degrees(angle)

def calculateCommand(frame):
    target, robot, green = getImportantInfo(frame)
    if target is None:
        return 0
    
    angle = calculateAngle(green, robot, target)

    if abs(angle) > 5: # error margin
        return angle
        
    else return 1

