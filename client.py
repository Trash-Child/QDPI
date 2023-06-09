import socket

def start_client(ip, port):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ip, port))
    return client_socket

def send_data(client_socket, data):
    client_socket.sendall(data.encode())
    reply = client_socket.recv(1024).decode()
    client_socket.close()
    return reply

def main():
    SERVER_IP = '192.168.43.184'  # Replace with your EV3's IP address.
    SERVER_PORT = 1234  # The same port used by the EV3's server.

    # Start the client and send some data.
    client_socket = start_client(SERVER_IP, SERVER_PORT)
    reply = send_data(client_socket, '1')
    print('Received:', reply)

if __name__ == '__main__':
    main()
