import socket
import cv2
from movementLogic import calculateCommand, getDistance, distanceToBall, calculateCommandToGoal
from Camera.BallAnalysis import analyseFrame
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
    encodedData = data.encode()  # Convert data to bytes
    total_sent = 0
    while total_sent < len(encodedData):
        sent = client_socket.send(encodedData[total_sent:])
        if sent == 0:  # Socket connection broken
            raise RuntimeError("Socket connection broken")
        total_sent += sent
    if data == '1':
        return None
    reply = client_socket.recv(1024).decode()
    return reply

# Function to stop capturing video
def stop_capture(vid):
    vid.release()
    cv2.destroyAllWindows()

def getManualCommand():
    cmd = ""
    while True:
        if cmd != None and cmd == '?':
            print("1: Forward \n-1: Reverse \n2: Open gate \nabove 5 or below -5: rotate given amount\nQ: stop the program and server")
        cmd = input("Enter command (? for help): ")
        try:
            cmd = int(cmd)
            return cmd
        except Exception as e:
            print("Not an integer")


def readFrame(vid):
    ret, frame = vid.read()
    ret2, debugFrame = vid.read()
    return frame, debugFrame

def calibrate():
    vid = cv2.VideoCapture(0)
    while True:
        ret, frame = vid.read()
        ret1, debugFrame = vid.read()
        NW, SE = analyseFrame(frame, debugFrame, True)
        print('NW:', NW, ' SE:', SE)

    
def main():
    manual = False
    if input("Press 1 for calibration: ") == '1':
        calibrate()
    else:
        vid = cv2.VideoCapture(0)

    SERVER_IP = '192.168.43.184'  # EV3's IP address. default: 192.168.43.184 #bertram server ip: 172.20.10.4
    SERVER_PORT = 1234  # The same port used by the EV3's server.
    client_socket = start_client(SERVER_IP, SERVER_PORT)
    cmd = ""
    noBalls = False
    while True:
        try:
            if noBalls:
                frame, debugFrame = readFrame(vid)
                cmd = calculateCommandToGoal(frame, debugFrame, False)
                cv2.imshow('debugFrame', debugFrame)
                cv2.waitKey(1)
                if cmd == 2:
                    noBalls = False
                
            elif not manual:
                frame, debugFrame = readFrame(vid)
                cmd = calculateCommand(frame, debugFrame)
                cv2.imshow('debugFrame', debugFrame)
                cv2.waitKey(1)
            else:
                cmd = getManualCommand()
            reply = send_data(client_socket, cmd)
            print('Sent:', cmd)
            if cmd == 1 and not noBalls:
                if manual:
                    dist = getManualCommand()
                else:
                    dist = distanceToBall(frame, debugFrame)
                reply = send_data(client_socket, dist)
                print('Sent:', dist)

            if cmd == 1 and noBalls:
                dist = calculateCommandToGoal(frame, debugFrame, True)
                reply = send_data(client_socket, dist)
                print('Sent', dist)

            if cmd == 2:
                noBalls = False
            print('Waiting for command execution...')
            while reply != 'Command executed':
                reply = client_socket.recv(1024).decode()
            print('Done')
            

        except Exception as e:
            if hasattr(e, 'winerror') or hasattr(e, 'errno'):
                if e.winerror == 10053 or e.errno == errno.WSAECONNRESET:
                    print('Connection was abruptly closed')
                    break
            if isinstance(e, TypeError) and str(e)=="'NoneType' object is not subscriptable":
                print("no target found")
                noBalls = True

            else:

                traceback.print_exc()


    stop_capture(vid)
    client_socket.close()

if __name__ == '__main__':
    main()
