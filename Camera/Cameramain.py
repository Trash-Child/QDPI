import cv2
import numpy as np

# Code made following guide https://www.geeksforgeeks.org/python-opencv-capture-video-from-camera/
vid = cv2.VideoCapture(0)

def camera():
    while(True):
        # Capture vid frame by frame
        ret, frame = vid.read()

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Calculate threshold value
        max_intensity = np.max(gray)
        threshold = int(0.8 * max_intensity)

        # Threshold the grayscale image
        _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

        # Display the binary image
        cv2.imshow('Binary', binary)

        # 'q' button to stop display
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    vid.release()
    cv2.destroyAllWindows()

camera()
