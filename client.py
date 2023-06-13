import socket
import cv2
from movementLogic import calculateCommand
import time
import errno

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
    return reply

def start_capture():
    vid = cv2.VideoCapture(0)
    return vid

def stop_capture(vid):
    vid.release()
    cv2.destroyAllWindows()

def main():
    vid = start_capture()

    SERVER_IP = '192.168.43.184'  # EV3's IP address. default: 192.168.43.184
    SERVER_PORT = 1234  # The same port used by the EV3's server.
    client_socket = start_client(SERVER_IP, SERVER_PORT)
    while True:
        try:
            ret, frame = vid.read()
            cmd = calculateCommand(frame)
            reply = send_data(client_socket, cmd)
            print('Sent: ', cmd)
            print('Recieved: ', reply)
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
            print(f"Exception occurred: ", e)
            continue
    stop_capture(vid)
    client_socket.close()

if __name__ == '__main__':
    main()
