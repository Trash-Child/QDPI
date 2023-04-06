import cv2
import numpy as np

# Code made following guide https://www.geeksforgeeks.org/python-opencv-capture-video-from-camera/
vid = cv2.VideoCapture(0)

def camera():
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
                if r > 10:  # Set your minimum radius threshold here
                    # Extract region of interest (ROI) within the circle
                    roi = frame[y-r:y+r, x-r:x+r]
                    print("ROI shape:", roi.shape)  # Debugging statement

                    # Check if ROI is not None and has non-empty shape
                    if roi is not None and roi.shape[0] > 0 and roi.shape[1] > 0:
                        # Convert ROI to grayscale
                        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

                        # Apply binary thresholding to extract white regions
                        _, thresh = cv2.threshold(roi_gray, 220, 255, cv2.THRESH_BINARY)

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
                            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                            cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)

        # Display the frame with detected circles
        cv2.imshow('frame', frame)

        # Press 's' key to stop display
        if cv2.waitKey(1) & 0xFF == ord('s'):
            break

    vid.release()
    cv2.destroyAllWindows()

camera()
