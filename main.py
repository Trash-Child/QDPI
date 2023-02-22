#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Initialising the brick
ev3 = EV3Brick()

# Initialising motor at port a
motor_a = Motor(Port.A)
motor_b = Motor(Port.B)

#####--FUNCTIONAL CODE--#####

#move straight for time duration. 

#duration specified in timesp, speed_sp necessary 
motor_a.speed_sp = 1000
motor_a.time_sp = 500
motor_a.run_timed()

motor_b.speed_sp = 1000
motor_b.time_sp = 500
motor_b.run_timed()