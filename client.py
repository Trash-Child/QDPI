import socket  # Importing socket library for network communication
import cv2  # Importing OpenCV library for video processing
from movementLogic import calculateCommand  # Importing calculateCommand function from custom module movementLogic
import time
import errno
import numpy as np

# Function to start the client and connect to the server
def start_client(SERVER_IP, SERVER_PORT):
    # Creating a TCP socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Connecting to the server at the specified IP address and port
    client_socket.connect((SERVER_IP, SERVER_PORT))
    print(f"Created socket: {client_socket}")  # Printing the socket object for debugging
    return client_socket  # Returning the socket object

# Function to send data to the server and receive a reply
def send_data(client_socket, data):
    data = str(data)  # Ensuring that data is a string
    data = data.encode()  # Encoding the string data to bytes
    total_sent = 0
    # Keep sending data until all is sent
    while total_sent < len(data):
        # Sending a chunk of data
        sent = client_socket.send(data[total_sent:])
        # If nothing is sent, socket connection is probably broken
        if sent == 0:
            raise RuntimeError("Socket connection broken")
        total_sent += sent
    # Receiving the reply from the server
    reply = client_socket.recv(1024).decode()
    return reply  # Returning the reply as a string

# Function to start capturing video from the default camera (camera index 0)
def start_capture():
    vid = cv2.VideoCapture(0)
    return vid

# Function to stop capturing video and release the resources
def stop_capture(vid):
    vid.release()
    cv2.destroyAllWindows()

def main():
    manual = False
    # Allowing the user to select manual mode
    if input("Press 1 for manual, or anything else to continue: ") == '1':
        manual = True
    else:
        vid = start_capture()

    # Defining the server IP and port
    SERVER_IP = '172.20.10.4'
    SERVER_PORT = 1234
    # Starting the client
    client_socket = start_client(SERVER_IP, SERVER_PORT)
    
    # Main loop
    while True:
        try:
            # If not in manual mode, capture a frame and calculate a command
            if not manual:
                ret, frame = vid.read()
                ret2, debugFrame = vid.read()
                cmd = calculateCommand(frame, debugFrame)

                # Convert numpy float to a regular Python float if necessary
                if isinstance(cmd, np.floating):
                    cmd = float(cmd)
            # In manual mode, allow user to enter commands
            else:
                cmd = input("Enter command (1 for drive, >5 for turn, 404 for nothing): ")
            
            # Send the command to the server and receive the reply
            reply = send_data(client_socket, str(cmd))
            print('Sent:', cmd)
            print('Received:', reply)
            
            # Wait for the server to execute the command
            if reply != 'Command executed':
                print('Waiting for command execution...')
                while True:
                    reply = client_socket.recv(1024).decode()
                    if reply == 'Command executed':
                        break

        # Handle exceptions, e.g., in case of a broken connection
        except Exception as e:
            # Check if the exception is due to a socket error
            if hasattr(e, 'winerror') or hasattr(e, 'errno'):
                # If connection is closed by the server, break the loop
                if e.winerror == 10053 or e.errno == errno.WSAECONNRESET:
                    print('Connection closed')
                    break
            print(f"Exception occurred:", e)
            continue  # Continue with the next iteration if an exception occurred
    
    # Clean up at the end
    stop_capture(vid)
    client_socket.close()

if __name__ == '__main__':
    main()  # Start the main function if the script is run as a standalone program
