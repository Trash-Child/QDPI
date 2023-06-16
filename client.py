import socket
import cv2
from movementLogic import calculateCommand
import time
import errno
import traceback

# Function to start the client and connect to the server
def start_client(SERVER_IP, SERVER_PORT):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((SERVER_IP, SERVER_PORT))
    print(f"Created socket: {client_socket}")  # Debug print
    return client_socket

# Function to send data to the server and receive a reply
def send_data(client_socket, data):
    if isinstance(data, int) or isinstance(data, float):
        data = round(data)
        data = str(data)
    data = data.encode()  # Convert data to bytes
    total_sent = 0
    while total_sent < len(data):
        sent = client_socket.send(data[total_sent:])
        if sent == 0:  # Socket connection broken
            raise RuntimeError("Socket connection broken")
        total_sent += sent
    reply = client_socket.recv(1024).decode()
    return reply

# Function to stop capturing video
def stop_capture(vid):
    vid.release()
    cv2.destroyAllWindows()

def main():
    manual = False
    if input("Press 1 for manual, or anything else to continue: ") == '1':
        manual = True
    else:
        vid = cv2.VideoCapture(0)

    SERVER_IP = '192.168.43.184'  # EV3's IP address. default: 192.168.43.184
    SERVER_PORT = 1234  # The same port used by the EV3's server.
    client_socket = start_client(SERVER_IP, SERVER_PORT)
    cmd = ""
    while True:
        try:
            if not manual:
                ret, frame = vid.read()
                ret2, debugFrame = vid.read()
                cmd = calculateCommand(frame, debugFrame)
                cv2.imshow('debugFrame', debugFrame)
                cv2.waitKey(1)
            else:
                while True:
                    if cmd != None and cmd == '?':
                        print("1: Forward \n-1: Reverse \n2: Open gate \nabove 5 or below -5: rotate given amount\nQ: stop the program and server")
                    cmd = input("Enter command (? for help): ")
                    try:
                        cmd = int(cmd)
                        break
                    except Exception as e:
                        print("Not an integer")
                        
            reply = send_data(client_socket, cmd)
            print('Sent:', cmd)
            print('Waiting for command execution...')
            while reply != 'Command executed':
                reply = client_socket.recv(1024).decode()
            print('Done')
            

        except Exception as e:
            if hasattr(e, 'winerror') or hasattr(e, 'errno'):
                if e.winerror == 10053 or e.errno == errno.WSAECONNRESET:
                    print('Connection was abruptly closed')
                    break
            traceback.print_exc()

    stop_capture(vid)
    client_socket.close()

if __name__ == '__main__':
    main()
