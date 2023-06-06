import cv2
import numpy as np
import time

# Code made following guide https://www.geeksforgeeks.org/python-opencv-capture-video-from-camera/
vid = cv2.VideoCapture(0)

def update_continuous_balls(circle, continuous_balls, position_error_margin, size_error_margin, counter_threshold=10, same_pos_counter_threshold=20):
    x, y, r = circle
    updated_continuous_balls = []
    found_match = False

    for prev_x, prev_y, prev_r, count, same_pos_count in continuous_balls:

        # Calculate the difference in position and size between the current circle and the previous circle
        position_diff = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        size_diff = abs(r - prev_r)

        # If the difference in position and size is within the error margins, consider it the same ball
        if position_diff <= position_error_margin and size_diff <= size_error_margin:

            # If the ball has not been matched before, add it to the list of continuous balls
            if not found_match:
                updated_continuous_balls.append((x, y, r, 0, same_pos_count+1 if position_diff == 0 else 0))
                found_match = True

            # If the ball has been matched before, update the continuous ball's position and size    
            else:
                updated_continuous_balls.append((prev_x, prev_y, prev_r, count, same_pos_count))
        else:
            if count < counter_threshold and same_pos_count < same_pos_counter_threshold:
                updated_continuous_balls.append((prev_x, prev_y, prev_r, count + 1, same_pos_count))

    # If the circle has not been matched, append it to the list
    if not found_match:
        updated_continuous_balls.append((x, y, r, 0, 0))

    return updated_continuous_balls



def line_intersection(line1, line2):
    # Unpack lines
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2

    # Compute determinants
    det1 = (x1 - x2) * (y3 - y4)
    det2 = (y1 - y2) * (x3 - x4)
    d = det1 - det2

    if d != 0:  # lines are not parallel
        px = ((x1*y2 - y1*x2) * (x3 - x4) - (x1 - x2) * (x3*y4 - y3*x4)) / d
        py = ((x1*y2 - y1*x2) * (y3 - y4) - (y1 - y2) * (x3*y4 - y3*x4)) / d
        return (px, py)
    else:
        return None



def detect_walls(frame):
    # Convert frame to grayscale for line detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply Canny edge detection
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)

    # Apply Probabilistic Hough Line Transform
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 10, minLineLength=100, maxLineGap=10)
    walls = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            # Add the wall's endpoints to the array
            walls.append(((x1, y1), (x2, y2)))

            # Draw the line on the colored frame
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
    
    intersections = []

    if lines is not None:
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                intersection = line_intersection(lines[i][0], lines[j][0])
                # check if intersection is within frame
                if intersection is not None and 0 <= intersection[0] < frame.shape[1] and 0 <= intersection[1] < frame.shape[0]:
                    intersections.append(intersection)

    # find corner points
    corners = []
    if intersections:
        nw = min(intersections, key=lambda p: p[0] + p[1])
        ne = max(intersections, key=lambda p: p[0] - p[1])
        sw = max(intersections, key=lambda p: p[1] - p[0])
        se = max(intersections, key=lambda p: p[0] + p[1])
        corners = [nw, ne, sw, se]
    
    return frame, walls, corners




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

        frame, walls, corners = detect_walls(frame)  # Use original colored frame for wall detection

        # Print corners
        print("Corners detected at:", corners)

        # Print walls
        for wall in walls:
            print("Wall detected from point {} to point {}".format(*wall))

        # detect circles in the gray frame
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=10, minRadius=5, maxRadius=10)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                # Filter out small circles
                # TODO: Filter out large circles as well
                if r > 0:  # Set your minimum radius threshold here
                    # Extract region of interest (ROI) within the circle
                    roi = frame[y-r:y+r, x-r:x+r]

                    if roi is not None and roi.shape[0] > 0 and roi.shape[1] > 0:
                        # Convert ROI to grayscale
                        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

                        # Apply adaptive binary thresholding to extract white regions
                        thresh = cv2.adaptiveThreshold(roi_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

                        # Count the number of white pixels in the thresholded image
                        white_pixels = cv2.countNonZero(thresh)

                        # Calculate the area of the circle
                        circle_area = np.pi * r**2

                        # Calculate the ratio of white pixels to circle area
                        ratio = white_pixels / circle_area

                        # Set a threshold for roundness
                        roundness_threshold = 0.8

                        # Set a threshold for circularity based on area comparison
                        area_threshold = 0.8

                        # If the ratio of white pixels to circle area is above the roundness threshold
                        # and the circle's area is above the area threshold, consider it a round object
                        if ratio > roundness_threshold and circle_area > area_threshold:
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
            print(corners)
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
