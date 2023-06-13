import cv2
import numpy as np
import time
import itertools
import math

vid = cv2.VideoCapture(0)

# function definition with error margins, counter threshold and same position counter threshold as parameters
def update_continuous_balls(circle, continuous_balls, position_error_margin, size_error_margin, counter_threshold=10, same_pos_counter_threshold=20):
    # unpacking circle tuple into x, y, and radius
    x, y, r = circle
    # initialize an empty list for updated balls
    updated_continuous_balls = []
    # flag to indicate if a match has been found in the continuous_balls list
    found_match = False

    # iterate over each ball in continuous_balls
    for prev_x, prev_y, prev_r, count, same_pos_count in continuous_balls:
        # calculate the difference in position (distance between the two circles)
        position_diff = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        # calculate the difference in size (absolute difference in radius)
        size_diff = abs(r - prev_r)

        # check if position and size differences are within error margins
        if position_diff <= position_error_margin and size_diff <= size_error_margin:
            # if a match hasn't been found yet
            if not found_match:
                # add the new circle to updated_balls with a reset counter and incremented same position counter (if position didn't change)
                updated_continuous_balls.append((x, y, r, 0, same_pos_count+1 if position_diff == 0 else 0))
                # set the found_match flag to True
                found_match = True
            else:
                # if a match was already found, add the old ball with its original counters
                updated_continuous_balls.append((prev_x, prev_y, prev_r, count, same_pos_count))
        else:
            # if the position and size differences are larger than error margins
            if count < counter_threshold and same_pos_count < same_pos_counter_threshold:
                # add the old ball with incremented counter, but only if it hasn't exceeded the counter thresholds
                updated_continuous_balls.append((prev_x, prev_y, prev_r, count + 1, same_pos_count))

    # if no match was found in continuous_balls list
    if not found_match:
        # add the new circle to updated_balls with both counters reset to 0
        updated_continuous_balls.append((x, y, r, 0, 0))

    # return the list of updated balls
    return updated_continuous_balls


def detect_walls(frame):
    # Convert frame to grayscale for line detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply edge detection
    edged = cv2.Canny(gray, 30, 60)

    # Perform a dilation and erosion to close gaps in between object edges
    edged = cv2.dilate(edged, None, iterations=1)
    edged = cv2.erode(edged, None, iterations=1)

    # Find contours in the edge map
    contours, _ = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # Define a tolerance level for angles
    angle_tolerance = 10

    # Initialize a list to store rectangle corners
    rectangles = []

    # Iterate over all found contours
    for cnt in contours:
        # Approximate contour to a polygon
        epsilon = 0.02 * cv2.arcLength(cnt, True)
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


    # Apply Hough Line Transform
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)
    walls = []
    if lines is not None:
        for rho, theta in lines[:,0]:
            a = np.cos(theta) # x part of unit vector
            b = np.sin(theta) # y part of unit vector
            x0 = a * rho # x coordinate for line's intersection with y-axis
            y0 = b * rho # y coordinate for line's intersection with x-axis

            # Find two far apart points (10000 pixels apart)
            x1 = int(x0 + 10000 * -b)
            y1 = int(y0 + 10000 * a)
            x2 = int(x0 - 10000 * -b)
            y2 = int(y0 - 10000 * a)

    if len(rectangles) != 4:
        return vectors

            # Draw the line on the colored frame
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
    
    return frame, walls




def detect_walls(frame, template):
    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply template matching
    result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)

    # Set a threshold for matches
    threshold = 0.8

    # Get the locations where the template matches the frame
    locations = np.where(result >= threshold)

    # Draw rectangles around the matching locations
    for pt in zip(*locations[::-1]):
        cv2.rectangle(frame, pt, (pt[0] + template.shape[1], pt[1] + template.shape[0]), (0, 0, 255), 2)

    return frame


def camera():
    continuous_balls = []

    # Initialize the editable error margins
    position_error_margin = 2
    size_error_margin = 1

    frame_counter = 0
    warm_up_period = 30

    # Load the wall template
    wall_template = cv2.imread('C:/three_week_picture.png', cv2.IMREAD_GRAYSCALE)



    while True:
        ret, frame = vid.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # convert to grayscale
        gray = cv2.GaussianBlur(gray, (35, 35), 0)  # reduce noise

        frame, walls = detect_walls(frame)  # Use original colored frame for wall detection

        # Print walls
        for wall in walls:
            print("Wall detected from point {} to point {}".format(*wall))

        # detect circles in the gray frame
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=10, minRadius=5, maxRadius=10)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                # Filter out small circles
                if r > 0:  # Set your minimum radius threshold here
                    # Update the continuous balls with changeable error margins
                    continuous_balls = update_continuous_balls((x, y, r), continuous_balls, position_error_margin,
                                                               size_error_margin)
                    cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                    cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)

        # Print the continuous balls' positions for debugging
        print([(x, y) for x, y, _, _, _ in continuous_balls])

        # Display the frame with detected circles and walls
        cv2.imshow('frame', frame)

        # Press 'q' key to stop display
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        frame_counter += 1
    vid.release()
    cv2.destroyAllWindows()


camera()
