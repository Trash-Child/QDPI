import cv2
import numpy as np
import time
from collections import Counter 
import threading

# Function to update the list of continuous balls based on the detected circle
def update_continuous_balls(circle, continuous_balls, position_error_margin, size_error_margin, counter_threshold=10, no_match_threshold=10):
    x, y, r = circle
    updated_continuous_balls = []
    found_match = False

    for prev_x, prev_y, prev_r, count, no_match_count in continuous_balls:
        # Calculate the difference in position and size between the current circle and the previous circle
        position_diff = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        size_diff = abs(r - prev_r)

        if position_diff <= position_error_margin and size_diff <= size_error_margin:
            if not found_match:
                updated_continuous_balls.append((x, y, r, count + 1, 0))
                found_match = True
            else:
                updated_continuous_balls.append((prev_x, prev_y, prev_r, count, 0))  # Reset no_match_count if there's a match
        else:
            if no_match_count + 1 < no_match_threshold:  # Increment no_match_count, and remove the ball if it exceeds the threshold
                updated_continuous_balls.append((prev_x, prev_y, prev_r, count, no_match_count + 1))

    if not found_match and all(np.sqrt((x - prev_x)**2 + (y - prev_y)**2) > r + prev_r for prev_x, prev_y, prev_r, _, _ in continuous_balls):
        updated_continuous_balls.append((x, y, r, 0, 0))  # Append the current circle as a new ball only if it does not overlap with any existing ball

    return updated_continuous_balls


def most_frequent_points(point_list, error_margin):
    counter = Counter(point_list)
    frequent_points = [point for point, freq in counter.items() if freq > error_margin]
    return frequent_points


def update_continuous_midpoints(midpoint, continuous_midpoints, position_error_margin, counter_threshold=10):
    x, y = midpoint
    updated_continuous_midpoints = []
    found_match = False
    for (prev_x, prev_y, count) in continuous_midpoints:
        position_diff = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        if position_diff <= position_error_margin:
            if not found_match:
                updated_continuous_midpoints.append((x, y, 0))
                found_match = True
            else:
                updated_continuous_midpoints.append((prev_x, prev_y, count))
        else:
            if count < counter_threshold:
                updated_continuous_midpoints.append((prev_x, prev_y, count + 1))

    if not found_match:
        updated_continuous_midpoints.append((x, y, 0))

    most_frequent = most_frequent_points([(x, y) for x, y, _ in updated_continuous_midpoints], position_error_margin)
    
    return updated_continuous_midpoints, most_frequent if most_frequent else (x, y)


def update_continuous_corners(corner, continuous_corners, position_error_margin, counter_threshold=20, same_pos_counter_threshold=20):
    x, y = corner
    updated_continuous_corners = []
    found_match= False
    for (prev_x, prev_y, count, same_pos_count) in continuous_corners:
        position_diff = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        if position_diff <= position_error_margin:
            if not found_match:
                updated_continuous_corners.append((x,y,0,same_pos_count+1 if position_diff == 0 else 0))
                found_match = True
            else:
                updated_continuous_corners.append((prev_x,prev_y,count,same_pos_count))
        else:
            if count < counter_threshold and same_pos_count<same_pos_counter_threshold:
                updated_continuous_corners.append((prev_x, prev_y, count + 1, same_pos_count))

    if not found_match:
        updated_continuous_corners.append((x, y, 0, 0))

    most_frequent = most_frequent_points([(x, y) for x, y, _, _ in updated_continuous_corners], position_error_margin)
    
    return updated_continuous_corners, most_frequent if most_frequent else (x, y)


def line_intersection(line1, line2):
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2

    det1 = (x1 - x2) * (y3 - y4)
    det2 = (y1 - y2) * (x3 - x4)
    d = det1 - det2

    if d != 0:  
        px = ((x1*y2 - y1*x2) * (x3 - x4) - (x1 - x2) * (x3*y4 - y3*x4)) / d
        py = ((x1*y2 - y1*x2) * (y3 - y4) - (y1 - y2) * (x3*y4 - y3*x4)) / d
        return (px, py)
    else:
        return None


def detect_walls(frame, debugFrame, continuous_corners, position_error_margin):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 10, minLineLength=100, maxLineGap=10)
    walls = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            walls.append(((x1, y1), (x2, y2)))
            cv2.line(debugFrame, (x1, y1), (x2, y2), (0, 0, 255), 2)

    intersections = []

    if lines is not None:
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                intersection = line_intersection(lines[i][0], lines[j][0])
                if intersection is not None:
                    intersection = (round(intersection[0]), round(intersection[1]))  # Round the pixel values here
                    if 0 <= intersection[0] < frame.shape[1] and 0 <= intersection[1] < frame.shape[0]:
                        intersections.append(intersection)

    corners = []

    nw, ne, sw, se = (0, 0), (0, 0), (0, 0), (0, 0) 

    most_frequent_nw = most_frequent_ne = most_frequent_sw = most_frequent_se = None

    if intersections:
        nw = min(intersections, key=lambda p: p[0] + p[1])
        ne = max(intersections, key=lambda p: p[0] - p[1])
        sw = max(intersections, key=lambda p: p[1] - p[0])
        se = max(intersections, key=lambda p: p[0] + p[1])

        continuous_corners["nw"], most_frequent_nw = update_continuous_corners(nw, continuous_corners["nw"], position_error_margin)
        continuous_corners["ne"], most_frequent_ne = update_continuous_corners(ne, continuous_corners["ne"], position_error_margin)
        continuous_corners["sw"], most_frequent_sw = update_continuous_corners(sw, continuous_corners["sw"], position_error_margin)
        continuous_corners["se"], most_frequent_se = update_continuous_corners(se, continuous_corners["se"], position_error_margin)

    return frame, walls, continuous_corners, most_frequent_nw, most_frequent_ne, most_frequent_sw, most_frequent_se

def locateColoredBall(frame, lowerRGB, upperRGB):
    lower_color = np.array(lowerRGB)
    upper_color = np.array(upperRGB)

    best_location = []
    best_area = 0
    best_circularity = 0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue

        circularity = 4*np.pi*(area / (perimeter**2))

        if area <= best_area and circularity <= best_circularity:
            continue
        
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        best_location = cX, cY
        best_area = area
        best_circularity = circularity

    return best_location

# Global variable to store the closest ball
closest_ball = [None]


def findRobot(frame, debugFrame):
    green_dot = locateColoredBall(frame, [50, 40, 40], [89, 255, 255])
    blue_dot = locateColoredBall(frame, [90, 40, 40], [105, 255, 255])
    if not green_dot or not blue_dot:
        print("robot not found")
        return None

    cv2.circle(debugFrame, green_dot, 5, [0, 255, 0], 4)
    cv2.circle(debugFrame, blue_dot, 5, [255, 0, 0], 4)
    cx = (green_dot[0] + blue_dot[0]) // 2
    cy = (green_dot[1] + blue_dot[1]) // 2
    heading = (np.arctan2(blue_dot[1] - green_dot[1], blue_dot[0] - green_dot[0]) * 180 / np.pi) % 360

    return np.array([cx, cy]), heading
'''
def isCloseToWall(cX, cY, NW, SE):
    if cX < NW[0] + 30 or cY < SE[1] + 30:
        cv2.circle(debugFrame, (cX, cY), int(r), (0, 0, 255), 2)
        return True
    if cX > most_frequent_se[0] - 30 or cY > most_frequent_se[1] - 30:
        cv2.circle(debugFrame, (cX, cY), int(r), (0, 0, 255), 2)
        return True

    return False #default case
'''

# Function to analyze the frame and locate balls
def analyseFrame(frame, debugFrame, calibrate):
    NW = (89, 4) # CHANGE THIS
    SE = (656, 445) # CHANGE THIS
    continuous_corners = {
        "nw": [], 
        "ne": [], 
        "sw": [], 
        "se": []
    }

    continuous_midpoints = {
        "w": [], 
        "e": []
    }
    
    global closest_ball
    continuous_balls = []
    position_error_margin = 25
    size_error_margin = 3
    orange_ball_location = None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)  # Filter out low light pixels
    _, walls, continuous_corners, most_frequent_nw, most_frequent_ne, most_frequent_sw, most_frequent_se = detect_walls(frame, debugFrame, continuous_corners, position_error_margin)
    cv2.imshow('thresh', thresh)
    cv2.waitKey(1)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
    for cnt in contours:
        # Find the center of the white part
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            r = np.sqrt(cv2.contourArea(cnt) / np.pi)
            if r < 3:
                continue
            '''
            if isCloseToWall(cX, cY, most_frequent_nw, most_frequent_se):
                continue
            '''
            continuous_balls = update_continuous_balls((cX, cY, r), continuous_balls, position_error_margin, size_error_margin)
            cv2.circle(debugFrame, (cX, cY), int(r), (0, 255, 0), 2)
            cv2.circle(debugFrame, (cX, cY), 2, (0, 0, 255), 3)

            if closest_ball[0] and cX == closest_ball[0][0] and cY == closest_ball[0][1]:
                cv2.circle(debugFrame, (cX, cY), int(r) + 10, (255, 255, 0), 4)  # Draw an extra circle around the closest ball
    
    orange_ball_location = locateColoredBall(frame, [20, 150, 150], [40, 255, 255])
    if orange_ball_location:
        orange_ball_location = None

    if most_frequent_nw and most_frequent_sw:
        mid_w = ((most_frequent_nw[0] + most_frequent_sw[0]) // 2, (most_frequent_nw[1] + most_frequent_sw[1]) // 2)
        continuous_midpoints["w"], mid_w = update_continuous_midpoints(mid_w, continuous_midpoints["w"], position_error_margin)
    
    if most_frequent_ne and most_frequent_se:
        mid_e = ((most_frequent_ne[0] + most_frequent_se[0]) // 2, (most_frequent_ne[1] + most_frequent_se[1]) // 2)
        continuous_midpoints["e"], mid_e = update_continuous_midpoints(mid_e, continuous_midpoints["e"], position_error_margin)
      
    if calibrate:
        return most_frequent_nw, most_frequent_se
    
    if NW and SE:
        mid_w = SE[0], NW[1]+SE[1]/2
    return mid_w, mid_e, continuous_balls, orange_ball_location


def locate_nearest_ball(continuous_balls, orange_ball_location, robot):
    global closest_ball

    if robot is None:
        print("Robot not found")
        return None
    
    closest_distance = float('inf')
    closest_ball = [None]
    
    for ball in continuous_balls:
        distance = ((robot[0] - ball[0])**2 + (robot[1] - ball[1])**2)**0.5
        if distance < closest_distance:
            closest_distance = distance
            closest_ball[0] = ball
    # Check orange ball if it exists
    if not orange_ball_location:
        return closest_ball

    distance = ((robot[0] - orange_ball_location[0])**2 + (robot[1] - orange_ball_location[1])**2)**0.5
    if distance < closest_distance:
        closest_distance = distance
        closest_ball[0] = orange_ball_location

    return closest_ball