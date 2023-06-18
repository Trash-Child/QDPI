import cv2
import numpy as np
import math


def detectX(frame, debugFrame):
    # Convert frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define color range for red color in HSV
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    # Threshold the HSV image to get only red colors
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    # Bitwise-AND mask and original image
    red_segment = cv2.bitwise_and(frame, frame, mask=mask)
    
    # Convert segmented image to grayscale
    gray = cv2.cvtColor(red_segment, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to reduce noise
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Apply edge detection
    edges = cv2.Canny(blur, 5, 15, apertureSize=3)
    
    # Define the region of interest
    height, width = frame.shape[:2]
    mask = np.zeros((height, width), dtype=np.uint8)
    roi_corners = np.array([[(width*3//20, height*3//20), (width*3//20, height*17//20), (width*17//20, height*17//20), (width*17//20, height*3//20)]], dtype=np.int32)
    channel_count = frame.shape[2]
    ignore_mask_color = (255,)*channel_count
    cv2.fillPoly(mask, roi_corners, ignore_mask_color)
    masked_edges = cv2.bitwise_and(edges, mask)
    
    # Apply Hough Line Transform on the region of interest
    lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi/180, threshold=23, minLineLength=50, maxLineGap=20)
    
    # Initialize list to hold unique lines
    unique_lines = []
    line_vectors = []

    # Check for unique lines based on their slope and intercept
    slope_threshold = 0.1  # Slope difference threshold
    intercept_threshold = 10.0  # Intercept difference threshold
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2-x1 == 0:  # Vertical line
                slope = float('inf')
            else:
                slope = (y2-y1)/(x2-x1)  # Calculate slope
            intercept = y1 - slope*x1  # Calculate intercept
            if not any(abs(slope - m) < slope_threshold and abs(intercept - c) < intercept_threshold for m, c in unique_lines):
                unique_lines.append((slope, intercept))
                
                # Save the line as two points that define the line
                line_points = ((x1, y1), (x2, y2))  
                line_vectors.append(line_points)
                cv2.line(debugFrame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # Returning line vectors
    return line_vectors

