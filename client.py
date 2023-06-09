import socket
import cv2
from movementLogic import calculateCommand

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
        sent = client_socket.send(data[total_sent:])
        if sent == 0:  # Socket connection broken
            raise RuntimeError("Socket connection broken")
        total_sent += sent
    reply = client_socket.recv(1024).decode()
    client_socket.close()
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
    while True:
        ret, frame = vid.read()
        cmd = calculateCommand(frame)
        
        reply = send_data(client_socket, cmd)
        print('Recieved:', reply)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    stop_capture(vid)


if __name__ == '__main__':
    main()
