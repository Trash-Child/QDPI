import cv2
import numpy as np
import time
from collections import Counter 
import threading

# Function to update the list of continuous balls based on the detected circle
def update_continuous_balls(circle, continuous_balls, position_error_margin, size_error_margin, counter_threshold=10, no_match_threshold=10):
    x, y, r = circle  # Getting the circle's center coordinates and radius
    updated_continuous_balls = []
    found_match = False

    # Compare the current circle with the previous circles (balls) stored
    for prev_x, prev_y, prev_r, count, no_match_count in continuous_balls:
        # Calculate the difference in position and size between the current circle and the previous circle
        position_diff = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        size_diff = abs(r - prev_r)

        # Check if the position and size differences are within the error margins
        if position_diff <= position_error_margin and size_diff <= size_error_margin:
            # If a match hasn't been found yet, update the matched ball's count and set found_match to True
            if not found_match:
                updated_continuous_balls.append((x, y, r, count + 1, 0))
                found_match = True
            else:
                # If a match has already been found, reset the no_match_count but don't increase the count
                updated_continuous_balls.append((prev_x, prev_y, prev_r, count, 0))
        else:
            # If there is no match, and the no_match_count hasn't exceeded the threshold, increment the no_match_count
            if no_match_count + 1 < no_match_threshold:
                updated_continuous_balls.append((prev_x, prev_y, prev_r, count, no_match_count + 1))

    # If no match has been found for the current circle, and it doesn't overlap with any existing ball,
    # append it as a new ball
    if not found_match and all(np.sqrt((x - prev_x)**2 + (y - prev_y)**2) > r + prev_r for prev_x, prev_y, prev_r, _, _ in continuous_balls):
        updated_continuous_balls.append((x, y, r, 0, 0))

    return updated_continuous_balls


# The function calculates the most frequently occurring points in a list, within an error margin.
def most_frequent_points(point_list, error_margin):
    counter = Counter(point_list)
    frequent_points = [point for point, freq in counter.items() if freq > error_margin]
    return frequent_points

# Function to update the list of continuous midpoints based on the detected midpoint
def update_continuous_midpoints(midpoint, continuous_midpoints, position_error_margin, counter_threshold=10):
    x, y = midpoint  # Getting the midpoint's coordinates
    updated_continuous_midpoints = []
    found_match = False
    # Compare the current midpoint with the previous midpoints stored
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

    return updated_continuous_midpoints

# Function to update the list of continuous corners based on the detected corner
def update_continuous_corners(corner, continuous_corners, position_error_margin, counter_threshold=10):
    x, y = corner  # Getting the corner's coordinates
    updated_continuous_corners = []
    found_match = False
    # Compare the current corner with the previous corners stored
    for (prev_x, prev_y, count) in continuous_corners:
        position_diff = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        if position_diff <= position_error_margin:
            if not found_match:
                updated_continuous_corners.append((x, y, 0))
                found_match = True
            else:
                updated_continuous_corners.append((prev_x, prev_y, count))
        else:
            if count < counter_threshold:
                updated_continuous_corners.append((prev_x, prev_y, count + 1))

    if not found_match:
        updated_continuous_corners.append((x, y, 0))

    return updated_continuous_corners

# Function to compute intersection point of two lines, each defined by two points (x1, y1) and (x2, y2).
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

# Function to detect walls in a frame and update the list of continuous corners
def detect_walls(frame, corners, continuous_corners):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Apply Canny edge detection
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    # Use Hough Line Transform to detect lines
    lines = cv2.HoughLines(edges, 1, np.pi/180, 200)

    new_corners = []

    # For each detected line
    for line in lines:
        rho, theta = line[0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        # Calculate the line's start and end points
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))

        # For each pair of lines, calculate the intersection point (corner)
        for other_line in lines:
            if other_line is line:
                continue
            other_rho, other_theta = other_line[0]
            other_a = np.cos(other_theta)
            other_b = np.sin(other_theta)
            other_x0 = other_a*other_rho
            other_y0 = other_b*other_rho
            other_x1 = int(other_x0 + 1000*(-other_b))
            other_y1 = int(other_y0 + 1000*(other_a))
            other_x2 = int(other_x0 - 1000*(-other_b))
            other_y2 = int(other_y0 - 1000*(other_a))

            try:
                corner = line_intersection(((x1, y1), (x2, y2)), ((other_x1, other_y1), (other_x2, other_y2)))
                new_corners.append(corner)
            except Exception:
                pass

    # Update the list of continuous corners based on the detected corners
    for corner in new_corners:
        continuous_corners = update_continuous_corners(corner, continuous_corners, 5)

    return continuous_corners

# Function to detect balls in a frame and update the list of continuous balls
def detect_balls(frame, continuous_balls):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Use Hough Circle Transform to detect circles
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=100, param1=50, param2=30, minRadius=20, maxRadius=100)
    
    # If circles are detected
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")  # Round off to integer values
        # For each detected circle, update the list of continuous balls
        for (x, y, r) in circles:
            continuous_balls = update_continuous_balls((x, y, r), continuous_balls, position_error_margin=20, size_error_margin=10)

    return continuous_balls
