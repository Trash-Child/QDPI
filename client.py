import socket
import cv2
from movementLogic import calculateCommand
import time
import errno
import threading

# Function to start the client and connect to the server
def start_client(SERVER_IP, SERVER_PORT):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((SERVER_IP, SERVER_PORT))
    print(f"Created socket: {client_socket}")  # Debug print
    return client_socket

# Function to send data to the server and receive a reply
def send_data(client_socket, data):
    if isinstance(data, int):
        data = str(data)  # Convert integer to string
    data = data.encode()  # Convert data to bytes
    total_sent = 0
    while total_sent < len(data):
        sent = client_socket.send(data[total_sent:])
        if sent == 0:  # Socket connection broken
            raise RuntimeError("Socket connection broken")
        total_sent += sent
    reply = client_socket.recv(1024).decode()
    return reply

# Function to start capturing video
def start_capture():
    vid = cv2.VideoCapture(0)
    return vid

# Function to stop capturing video
def stop_capture(vid):
    vid.release()
    cv2.destroyAllWindows()


# Function for continous vid read
def continousFrame(vid):
    while True:
        ret, frame = vid.read()
        if not ret:
            break
        
        with frame_lock:
            global latest_frame # Declare as a global variable
            latest_frame = latest_frame.copy()  # Store the latest frame

# Function for getting the latest frame from continous frame
def get_frame():
    with frame_lock:
        return latest_frame.copy() if latest_frame is not None else None


def main():
    global latest_frame, frame_lock  # Declare frame and frame_lock as global
    latest_frame = None 
    frame_lock = threading.Lock()  # A lock to prevent race conditions
    vid = None 

    if input("Press 1 for manual, or anything else to continue: ") == '1': 
        manual = True # Run in manual mode
    else:
        vid = start_capture() # Run in automatic mode

    SERVER_IP = '192.168.43.184'  # EV3's IP address. default: 192.168.43.184
    SERVER_PORT = 1234  # The same port used by the EV3's server.
    client_socket = start_client(SERVER_IP, SERVER_PORT)
    
    while True:
        try:
            if not manual:
                # Capture continous video feed in a thread
                vid_thread = threading.Thread(target=continousFrame, args=(vid,))
                vid_thread.start()
                
                # Get latest frame to use for movement logic
                frame = get_frame()
                cmd = calculateCommand(frame)
            else:
                cmd = input("Enter command (1 for drive, >5 for turn, 404 for nothing): ")
            
            reply = send_data(client_socket, cmd)
            print('Sent:', cmd)
            print('Received:', reply)
            
            if reply != 'Command executed':
                print('Waiting for command execution...')
                while True:
                    reply = client_socket.recv(1024).decode()
                    if reply == 'Command executed':
                        break

        except Exception as e:
            if hasattr(e, 'winerror') or hasattr(e, 'errno'):
                if e.winerror == 10053 or e.errno == errno.WSAECONNRESET:
                    print('Connection closed')
                    break
            print(f"Exception occurred:", e)
            continue
    
    # Stop video capture if not manual
    if vid is not None:
        stop_capture(vid)

    # Close the socket
    client_socket.close()

if __name__ == '__main__':
    main()
