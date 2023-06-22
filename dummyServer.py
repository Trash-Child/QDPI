import socket

# Function to start the server and bind it to a specific port
# Anton 100%
def start_server(port):
    print("Starting server...")
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('', port))
    server_socket.listen(1)
    print("Server started.")
    return server_socket

# Function to send a reply to the client
# Anton 100%
def send_reply(client_socket, message):
    try:
        while message:
            bytes_sent = client_socket.send(message)
            message = message[bytes_sent:]
    except Exception as e:
        print('Error: {}'.format(e))

# Function to handle the client's requests
# Anton 100%
def handle_client(client_socket):
    while True:
        data = client_socket.recv(1024)
        if not data:
            break
        reply = data.decode()
        send_reply(client_socket, handle_data(data))
        print('Server received:', reply)
    client_socket.close()

# Function to handle the data received from the client and execute the corresponding command
# Anton 100%
def handle_data(data):
    data = int(data)
    print("Data: ", data)
    return "Command executed"

# Function to run the server and handle client connections
# Anton 100%
def run_server():
    port = 1234
    server_socket = start_server(port)

    while True:
        print("Waiting for client...")
        client_socket, client_address = server_socket.accept()  # Wait for a new client connection
        print("Client connected:", client_address)
        data = handle_client(client_socket)
        print('Server received:', data)

# Uncomment the following two lines if running the server in a separate thread
# server_thread = Thread(target=run_server)
# server_thread.start()

# Run the server in the main thread
run_server()
