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

# Initialising motor at port b
motor_b = Motor(Port.B)

# Initialising motor at port c
motor_c = Motor(Port.C)

# Initialising motor at port d
motor_d = Motor(Port.D)


#####--FUNCTIONAL CODE--#####

#run motor at 500 degress pr second
motor_a.run_target(500)