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

ev3 = brick()
motor_r = Motor(Port.A)
motor_l = Motor(Port.B)
motor_frontWheels = Motor(Port.C)
motor_v = Motor(Port.D)
gyro = GyroSensor(Port.S1)

qdpi = DriveBase(motor_l, motor_r, wheel_diameter = 32.5, axle_track = 204.5)


def start_server(port):
    print("Starting server...")
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('', port))
    server_socket.listen(1)
    print("Server started.")
    return server_socket

def handle_client(client_socket):
    data = client_socket.recv(1024)
    reply = data.decode()
    try:
        message = b'Thank you for connecting'
        while message:
            bytes_sent = client_socket.send(message)
            message = message[bytes_sent:]
    except Exception as e:
        print('Error: {}'.format(e))
    client_socket.close()  # Close the client socket after handling the data
    return reply

    

def handle_data(data):
    data = int(data)
    if data == 404:
        print("An error occured")
    elif data <= 5:
        correction = (0 - gyro.angle())*1
        qdpi.drive(150, correction) # 150 = base speed
    
    elif data > 5:
        qdpi.drive(0,data) # turn
    

def run_server():
    port = 1234
    server_socket = start_server(port)

    while True:  # Continuously accept new connections.
        client_socket, client_address = server_socket.accept()  # Wait for a new client connection
        print("Client connected:", client_address)
        data = handle_client(client_socket)
        motor_frontWheels.run(500)
        gyro.reset_angle(0)
        handle_data(data)
        print('Server received:', data)


print("Starting server thread...")
server_thread = Thread(target=run_server)
server_thread.start()

# Add a delay to keep the main program running for a while
# and prevent it from ending immediately.
import time
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Shutting down...")
print("End of main program.")
