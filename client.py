import socket
import cv2
from Camera.BallAnalysis import runCamera
from movementLogic import calculateCommand

def start_client(ip, port):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ip, port))
    return client_socket

def send_data(client_socket, data):
    client_socket.sendall(data.encode())
    reply = client_socket.recv(1024).decode()
    client_socket.close()
    return reply

def start_capture():
    vid = cv2.VideoCapture()
    return vid

def stop_capture(vid):
    vid.release()
    cv2.destroyAllWindows()

def establishConnection():
    SERVER_IP = '192.168.43.184'  # Replace with your EV3's IP address.
    SERVER_PORT = 1234  # The same port used by the EV3's server.

    # Start the client and send some data.
    client_socket = start_client(SERVER_IP, SERVER_PORT)
    reply = send_data(client_socket, calculateCommand(frame))
    print('Received:', reply)

if __name__ == '__main__':
    main()
