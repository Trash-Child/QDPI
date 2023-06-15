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


# Function to locate the orange ball in the frame
def locateOrangeBall(frame):
    lower_orange = np.array([10, 100, 100])
    upper_orange = np.array([20, 255, 255])

    best_location = []
    best_area = 0
    best_circularity = 0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
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


def findRobot(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 90, 255, cv2.THRESH_BINARY)
    thresh = cv2.bitwise_not(thresh)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.imshow('thresh', thresh)
    cv2.waitKey(1)
    if contours and len(contours) >= 2:
        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
        max_contour = sorted_contours[0]
        M1 = cv2.moments(max_contour)
        
        if M1['m00'] != 0:
            cx1 = int(M1['m10'] / M1['m00'])
            cy1 = int(M1['m01'] / M1['m00'])

            cv2.circle(thresh, (cx1, cy1), 5, (0, 0, 255), -1)
            max_distance = np.inf
            second_max_contour = None
            min_dist = 0

            # Find the contour closest to the max_contour
            for contour in sorted_contours[1:]:
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    dist = np.sqrt((cx1 - cx)**2 + (cy1 - cy)**2)
                    if dist < min_dist and dist < max_distance:
                        min_dist = dist
                        second_max_contour = contour

            if second_max_contour is not None:
                M2 = cv2.moments(second_max_contour)
                cx2 = int(M2['m10'] / M2['m00'])
                cy2 = int(M2['m01'] / M2['m00'])
                
                cv2.circle(thresh, (cx2, cy2), 5, (0, 0, 255), -1)
                cv2.line(thresh, (cx1, cy1), (cx2, cy2), (255, 0, 0), 2)
                
                heading = np.arctan2(cy2 - cy1, cx2 - cx1) * 180 / np.pi
                #print("Center of robot: ({}, {})".format(cx1, cy1))
                #print("Heading: {} degrees".format(heading))
                return np.array([cx1, cy1]), heading
            
    #else:
       #print("No dark spot found")
    return None


# Function to analyze the frame and locate balls
def analyseFrame(frame):
    global closest_ball
    continuous_balls = []
    position_error_margin = 25
    size_error_margin = 3
    orange_ball_location = None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)  # Filter out low light pixels

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
    for cnt in contours:
        # Find the center of the white part
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            r = np.sqrt(cv2.contourArea(cnt) / np.pi)
            
            continuous_balls = update_continuous_balls((cX, cY, r), continuous_balls, position_error_margin, size_error_margin)
            cv2.circle(frame, (cX, cY), int(r), (0, 255, 0), 2)
            cv2.circle(frame, (cX, cY), 2, (0, 0, 255), 3)

            if closest_ball[0] and cX == closest_ball[0][0] and cY == closest_ball[0][1]:
                cv2.circle(frame, (cX, cY), int(r) + 10, (255, 0, 0), 4)  # Draw an extra circle around the closest ball
    
        orange_ball_location = locateOrangeBall(frame)
        if orange_ball_location:
            cv2.circle(frame, (cX, cY), int(r), (255, 127, 0), 4)

    return continuous_balls, orange_ball_location, robot


# Function to locate the nearest ball to the robot among the continuous balls and orange ball
def locate_nearest_ball(continuous_balls, orange_ball_location, green_dot, blue_dot):
    global closest_ball

    if not green_dot or not blue_dot:
        print("Robot not found")
        return None

    # Define robot as the middle of the green dot and blue dot
    robot = ((green_dot[0] + blue_dot[0]) / 2, (green_dot[1] + blue_dot[1]) / 2)
    
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

    return closest_ball, robot


vid = cv2.VideoCapture(0)
while True:
    ret, frame = vid.read()
    findRobot(frame)
    cv2.imshow('frame', frame)
    cv2.waitKey(1)