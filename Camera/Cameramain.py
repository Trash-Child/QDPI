import cv2

#Code made following guide https://www.geeksforgeeks.org/python-opencv-capture-video-from-camera/
vid = cv2.VideoCapture(0)

def camera():
    while(True):

        #Capture vid frame by frame
        ret, frame=vid.read()

       #display of frame
        cv2.imshow('frame', frame)

       #s button to stop display
        if cv2.waitKey(1) & 0xFF == ord('s'):
            break
    

    vid.release()
    cv2.destroyAllWindows()


camera()


