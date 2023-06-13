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


def send_reply(client_socket, message):
    try:
        while message:
            bytes_sent = client_socket.send(message)
            message = message [bytes_sent:]
    except Exception as e:
        print('Error: {}'.format(e))

def handle_client(client_socket):
    data = client_socket.recv(1024)
    reply = data.decode()
    send_reply(client_socket, handle_data(data))
    client_socket.close()
    return reply


def handle_data(data):
    data = int(data)
    
    if data == 404:
        print("An error occured")
        return "Error"
    elif data == 1:
        print("going straight")
        correction = (0 - gyro.angle())*1
        qdpi.drive(150, correction)
        wait(1000)
        qdpi.stop()
        return "forward"
    elif data > 5 or data < -5:
        print("turning ", data)
        qdpi.turn(data) # turn
        return "turning"
    return "Data Error"

def run_server():
    port = 1234
    server_socket = start_server(port)

    while True:
        client_socket, client_address = server_socket.accept()  # Wait for a new client connection
        print("Client connected:", client_address)
        motor_frontWheels.run(500)
        gyro.reset_angle(0)
        data = handle_client(client_socket)

        print('Server received:', data)


print("Starting server thread...")
server_thread = Thread(target=run_server)
server_thread.start()

