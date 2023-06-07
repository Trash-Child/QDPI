import cv2
import numpy as np
import time
import itertools
import math

vid = cv2.VideoCapture(0)

def update_continuous_balls(circle, continuous_balls, position_error_margin, size_error_margin, counter_threshold=10, same_pos_counter_threshold=20):
    x, y, r = circle
    updated_continuous_balls = []
    found_match = False

    for prev_x, prev_y, prev_r, count, same_pos_count in continuous_balls:

        position_diff = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        size_diff = abs(r - prev_r)

        if position_diff <= position_error_margin and size_diff <= size_error_margin:
            if not found_match:
                updated_continuous_balls.append((x, y, r, 0, same_pos_count+1 if position_diff == 0 else 0))
                found_match = True
            else:
                updated_continuous_balls.append((prev_x, prev_y, prev_r, count, same_pos_count))
        else:
            if count < counter_threshold and same_pos_count < same_pos_counter_threshold:
                updated_continuous_balls.append((prev_x, prev_y, prev_r, count + 1, same_pos_count))

    if not found_match:
        updated_continuous_balls.append((x, y, r, 0, 0))

    return updated_continuous_balls

def line_intersection(line1, line2):
    x1, y1, x2, y2 = line1[0]
    x3, y3, x4, y4 = line2[0]

    det1 = (x1 - x2) * (y3 - y4)
    det2 = (y1 - y2) * (x3 - x4)
    d = det1 - det2

    if d != 0:  # lines are not parallel
        px = ((x1*y2 - y1*x2) * (x3 - x4) - (x1 - x2) * (x3*y4 - y3*x4)) / d
        py = ((x1*y2 - y1*x2) * (y3 - y4) - (y1 - y2) * (x3*y4 - y3*x4)) / d
        return (px, py)
    else:
        return None

def is_rectangle(p1, p2, p3, p4):
    cx = (p1[0] + p2[0] + p3[0] + p4[0]) / 4
    cy = (p1[1] + p2[1] + p3[1] + p4[1]) / 4

    dd1 = (cx - p1[0])**2 + (cy - p1[1])**2
    dd2 = (cx - p2[0])**2 + (cy - p2[1])**2
    dd3 = (cx - p3[0])**2 + (cy - p3[1])**2
    dd4 = (cx - p4[0])**2 + (cy - p4[1])**2

    return abs(dd1 - dd2) < 1e-9 and abs(dd1 - dd3) < 1e-9 and abs(dd1 - dd4) < 1e-9

def detect_and_draw_rectangle(frame):
    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply edge detection
    edged = cv2.Canny(gray, 50, 100)

    # Perform a dilation and erosion to close gaps in between object edges
    edged = cv2.dilate(edged, None, iterations=1)
    edged = cv2.erode(edged, None, iterations=1)

    # Find contours in the edge map
    contours, _ = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # Define a tolerance level for angles
    angle_tolerance = 15

    # Initialize a list to store rectangle corners
    rectangles = []

    # Iterate over all found contours
    for cnt in contours:
        # Approximate contour to a polygon
        epsilon = 0.02*cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        # If the polygon has four points, it is assumed to be a rectangle
        if len(approx) == 4:
            is_rect = True
            for i in range(4):
                # Calculate the angle of the corner
                p0, p1, p2 = approx[i - 1], approx[i], approx[(i + 1) % 4]
                angle = abs(math.degrees(math.atan2(p2[0][1] - p1[0][1], p2[0][0] - p1[0][0]) -
                            math.atan2(p0[0][1] - p1[0][1], p0[0][0] - p1[0][0])))

                # Adjust angles to be between 0 and 180
                angle = angle if angle < 180 else 360 - angle

                # If the angle deviates too much from 90, it's not a rectangle
                if not 90 - angle_tolerance <= angle <= 90 + angle_tolerance:
                    is_rect = False
                    break

            if is_rect:  # Only if it's a rectangle
                cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
                # Add rectangle corners to the list
                rectangles.append(approx)

    return frame, rectangles


def camera():
    continuous_balls = []

    # Initialize the editable error margins
    position_error_margin = 2
    size_error_margin = 1

    frame_counter = 0
    warm_up_period = 30

    while True:
        ret, frame = vid.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # convert to grayscale
        gray = cv2.GaussianBlur(gray, (5, 5), 0) # reduce noise

        frame, rectangles = detect_and_draw_rectangle(frame)

        # Print corners
        for rect in rectangles:
            print("Rectangle detected at:", rect)

        # detect circles in the gray frame
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=10, minRadius=5, maxRadius=10)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                # Filter out small circles
                if r > 0:  # Set your minimum radius threshold here
                    # Update the continuous balls with changable error margins
                    continuous_balls = update_continuous_balls((x, y, r), continuous_balls, position_error_margin, size_error_margin)
                    cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                    cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)

        # Print the continuous balls' positions for debugging
        print([(x, y) for x, y, _, _, _ in continuous_balls])

        # Display the frame with detected circles
        cv2.imshow('frame', frame)

        # Press 'q' key to stop display
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print(rectangles)
            break

        # Increase position error margin with 'w' key
        elif key == ord('w'):
            position_error_margin += 1
            print("Position error margin:", position_error_margin)

        # Decrease position error margin with 's' key
        elif key == ord('s'):
            position_error_margin += -1
            print("Position error margin:", position_error_margin)

        frame_counter += 1
    vid.release()
    cv2.destroyAllWindows()

camera()

