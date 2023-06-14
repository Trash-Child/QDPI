import socket
import cv2
from movementLogic import calculateCommand
import time
import errno

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

def main():
    manual = False
    if input("Press 1 for manual, or anything else to continue: ") == '1':
        manual = True
    else:
        vid = start_capture()

    SERVER_IP = '172.20.10.13'  # EV3's IP address. default: 192.168.43.184
    SERVER_PORT = 1234  # The same port used by the EV3's server.
    client_socket = start_client(SERVER_IP, SERVER_PORT)
    cmd = ""
    while True:
        try:
            if not manual:
                ret, frame = vid.read()
                cmd = calculateCommand(frame)
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
    
    stop_capture(vid)
    client_socket.close()

if __name__ == '__main__':
    main()
