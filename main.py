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
        send_reply(client_socket, handle_data(data))
    client_socket.close()

# This function handles the data received from the client and executes the corresponding command
def handle_data(data):

    # Try to convert data to float and round it to the nearest integer.
    # This is necessary because the client may send a string representation of a float number.
    try:
        data = round(float(data))  
    except ValueError:
        # If the conversion fails, print an error message and return a string indicating an error.
        print("Received data could not be converted to integer.")
        return "Data error"

    # Wait for a second. This is often used to give some time for a device to initialize or settle after 
    # performing a command. 
    wait(1000)
    
    # Depending on the value of data, perform different actions.
    if data == 404:
        # If data equals 404, print an error message and return a string indicating that the command has been executed.
        print("Error 404")
        return "Command executed"

    elif data == 1:
        # If data equals 1, make the robot go straight and return a string indicating that the command has been executed.
        print("going straight")
        qdpi.straight(200)
        return "Command executed"

    elif data == -1:
        # If data equals -1, make the robot reverse and return a string indicating that the command has been executed.
        print("Reversing")
        qdpi.straight(-100)
        return "Command executed"

    elif data >= 5 or data <= -5:
        # If data equals or exceeds 5 or -5, make the robot turn to the specified angle 
        # and return a string indicating that the command has been executed.
        print("Turning", data)
        qdpi.turn(data)
        return "Command executed"

    elif data == 2:
        # If data equals 2, perform a sequence of motor actions 
        # and return a string indicating that the command has been executed.
        motor_v.run(450)
        wait(200)
        motor_v.run(0)
        wait(2000)
        motor_v.run(-450)
        wait(200)
        motor_v.run(0)
        return "Command executed"

    # If data does not match any of the expected values, print an error message and return a string 
    # indicating that the command has been executed.
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
