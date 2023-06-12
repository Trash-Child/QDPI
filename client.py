import socket
import cv2
from movementLogic import calculateCommand
import time

def start_client(SERVER_IP, SERVER_PORT):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((SERVER_IP, SERVER_PORT))
    print(f"Created socket: {client_socket}")  # Debug print
    return client_socket


def send_data(client_socket, data):
    if isinstance(data, int):
        data = str(data)  # Convert integer to string
    data = data.encode()  # Convert data to bytes
    total_sent = 0
    while total_sent < len(data):
        print(len(data))
        sent = client_socket.send(data[total_sent:])
        print("sent data", sent)
        if sent == 0:  # Socket connection broken
            raise RuntimeError("Socket connection broken")
        total_sent += sent
    reply = client_socket.recv(1024).decode()
    return reply

def start_capture():
    vid = cv2.VideoCapture(0)
    return vid

def stop_capture(vid):
    vid.release()
    cv2.destroyAllWindows()

def main():
    vid = start_capture()
    SERVER_IP = '192.168.43.184'  # EV3's IP address.
    SERVER_PORT = 1234  # The same port used by the EV3's server.
    client_socket = start_client(SERVER_IP, SERVER_PORT)
    try:  # Add a try block to handle potential exceptions
        while True:
            ret, frame = vid.read()
            cmd = calculateCommand(frame)
            print("calc cmd: ", cmd)
            reply = send_data(client_socket, cmd)
            print('Received:', reply)
            cv2.imshow('frame', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    except Exception as e:  # Catch and print any exceptions
        print(f"Exception occurred: {e}")
    finally:  # Ensure that the socket and video capture are always closed
        client_socket.close()
        stop_capture(vid)

if __name__ == '__main__':
    main()
