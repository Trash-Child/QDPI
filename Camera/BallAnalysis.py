import cv2
import numpy as np

# Code made following guide https://www.geeksforgeeks.org/python-opencv-capture-video-from-camera/
vid = cv2.VideoCapture(0)

def update_continuous_balls(circle, continuous_balls, position_error_margin, size_error_margin):
    x, y, r = circle
    updated = False
    for idx, (prev_x, prev_y, prev_r) in enumerate(continuous_balls):
        position_diff = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        size_diff = abs(r - prev_r)

        if position_diff <= position_error_margin and size_diff <= size_error_margin:
            continuous_balls[idx] = (x, y, r)
            updated = True
            break

    if not updated:
        continuous_balls.append((x, y, r))
    
    return continuous_balls


def camera():
    continuous_balls = []

    # Initialize the editable error margins
    position_error_margin = 10
    size_error_margin = 3

    frame_counter = 0
    warm_up_period = 30

    while True:

        # Capture video frame by frame
        ret, frame = vid.read()

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply GaussianBlur to reduce noise
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # Detect circles using Hough Circle Transform
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                # Filter out small circles
                # TODO: Filter out large circles as well
                if r > 10:  # Set your minimum radius threshold here
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

                            if frame_counter > warm_up_period:
                                if len(continuous_balls) > 10:
                                    continuous_balls = []
                                
                                if len(continuous_balls) > 1:
                                    position_error_margin += 1
                                    size_error_margin += 1
                                elif len(continuous_balls) < 1:
                                    position_error_margin = max(1, position_error_margin - 1)
                                    size_error_margin = max(1, size_error_margin - 1)

        # Print the continuous balls' positions for debugging
        print([(x, y) for x, y, _ in continuous_balls])

        # Display the frame with detected circles
        cv2.imshow('frame', frame)

        # Press 'q' key to stop display
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        # Increase position error margin with 'p' key
        elif key == ord('p'):
            position_error_margin += 1
            print("Position error margin:", position_error_margin)

        # Decrease position error margin with 'o' key
        elif key == ord('o'):
            position_error_margin -= 1
            print("Position error margin:", position_error_margin)

        # Increase size error margin with 's' key
        elif key == ord('s'):
            size_error_margin += 1
            print("Size error margin:", size_error_margin)

        # Decrease size error margin with 'a' key
        elif key == ord('a'):
            size_error_margin -= 1
            print("Size error margin:", size_error_margin)

        frame_counter += 1
    vid.release()
    cv2.destroyAllWindows()

camera()
