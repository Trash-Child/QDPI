#!/usr/bin/env pybricks-micropython
from threading import Thread
from pybricks.hubs import EV3Brick as brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math
import socket

# Create instances of EV3 brick, motors, and sensors
ev3 = brick()
motor_r = Motor(Port.A)
motor_l = Motor(Port.B)
motor_frontWheels = Motor(Port.C)
motor_v = Motor(Port.D)
# gyro = GyroSensor(Port.S1)

# Create DriveBase instance for differential drive
qdpi = DriveBase(motor_l, motor_r, wheel_diameter=32.5, axle_track=204.5)

# Function to start the server and bind it to a specific port
def start_server(port):
    print("Starting server...")
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('', port))
    server_socket.listen(1)
    print("Server started.")
    return server_socket

# Function to send a reply to the client
def send_reply(client_socket, message):
    try:
        while message:
            bytes_sent = client_socket.send(message)
            message = message[bytes_sent:]
    except Exception as e:
        print('Error: {}'.format(e))

# Function to handle the client's requests
def handle_client(client_socket):
    while True:
        data = client_socket.recv(1024)
        if not data:
            break
        reply = data.decode()
        print('Server received:', reply)
        send_reply(client_socket, handle_data(data, client_socket))
    client_socket.close()


def getDist(client_socket):
    print("Waiting for dist...")
    dist = client_socket.recv(1024)
    dist = dist.decode()
    dist = int(dist)
    if not dist:
        print("dist error")
        return 0
    if dist < 250:
        return 300
    else:
        return (dist*26/10)-100

# Function to handle the data received from the client and execute the corresponding command
def handle_data(data, client_socket):
    data = int(data)
    wait(1000)
    
    if data == 404:
        print("Error 404")
        return "Command executed"

    elif data == 1:
        qdpi.straight(int(getDist(client_socket)))
        return "Command executed"

    elif data == -1:
        print("Reversing")
        qdpi.straight(-100)
        return "Command executed"

    elif data >= 5 or data <= -5:
        print("Turning", data)
        qdpi.turn(data)
        return "Command executed"

    elif data == 2:
        motor_v.run(450)
        wait(200)
        motor_v.run(0)
        wait(2000)
        motor_v.run(-450)
        wait(200)
        motor_v.run(0)
        qdpi.turn(360)
        return "Command executed"

    print("Data error")
    return "Command executed"

# Function to run the server and handle client connections
def run_server():
    port = 1234
    server_socket = start_server(port)
    motor_frontWheels.run(500)
    while True:
        print("Waiting for client...")
        client_socket, client_address = server_socket.accept()  # Wait for a new client connection
        print("Client connected:", client_address)
        # gyro.reset_angle(0)
        data = handle_client(client_socket)
        print('Server received:', data)

# Uncomment the following two lines if running the server in a separate thread
# server_thread = Thread(target=run_server)
# server_thread.start()

# Run the server in the main thread
run_server()
