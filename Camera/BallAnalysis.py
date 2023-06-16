import cv2
import numpy as np
import time
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


def locateColoredBall(frame, lowerRGB, upperRGB):
    lower_color = np.array(lowerRGB)
    upper_color = np.array(upperRGB)

    best_location = []
    best_area = 0
    best_circularity = 0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    cv2.imshow('mask', mask)
    cv2.waitKey(1)
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
    green_dot = locateColoredBall(frame, [36, 40, 40], [89, 255, 255])
    blue_dot = locateColoredBall(frame, [90, 40, 40], [99, 255, 255])
    if not green_dot or not blue_dot:
        print("robot not found")
        return None

    cv2.circle(debugFrame, green_dot, 5, [0, 255, 0], 4)
    cv2.circle(debugFrame, blue_dot, 5, [255, 0, 0], 4)
    cx = (green_dot[0] + blue_dot[0]) // 2
    cy = (green_dot[1] + blue_dot[1]) // 2
    heading = (np.arctan2(blue_dot[1] - green_dot[1], blue_dot[0] - green_dot[0]) * 180 / np.pi +180) % 360

    return np.array([cx, cy]), heading

# Function to analyze the frame and locate balls
def analyseFrame(frame, debugFrame):
    global closest_ball
    continuous_balls = []
    position_error_margin = 25
    size_error_margin = 3
    orange_ball_location = None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)  # Filter out low light pixels
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
            if r < 8:
                continue
            continuous_balls = update_continuous_balls((cX, cY, r), continuous_balls, position_error_margin, size_error_margin)
            cv2.circle(debugFrame, (cX, cY), int(r), (0, 255, 0), 2)
            cv2.circle(debugFrame, (cX, cY), 2, (0, 0, 255), 3)

            if closest_ball[0] and cX == closest_ball[0][0] and cY == closest_ball[0][1]:
                cv2.circle(debugFrame, (cX, cY), int(r) + 10, (255, 0, 0), 4)  # Draw an extra circle around the closest ball
    
        orange_ball_location = locateColoredBall(frame, [10, 100, 100], [20, 255, 255])
        if orange_ball_location:
            cv2.circle(debugFrame, orange_ball_location, 5, (255, 127, 0), 4)

    return continuous_balls, orange_ball_location


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