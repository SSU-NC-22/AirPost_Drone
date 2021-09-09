#!/usr/bin/env python3

import Jetson.GPIO as GPIO
import time


class DCMotor():
    def __init__(self):
        self.MOTOR_B_A1 = int(23)
        self.MOTOR_B_B1 = int(24)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.MOTOR_B_A1, GPIO.OUT)
        GPIO.setup(self.MOTOR_B_B1, GPIO.OUT)

        GPIO.output(self.MOTOR_B_A1, GPIO.LOW)
        GPIO.output(self.MOTOR_B_B1, GPIO.LOW)

    def stop(self):
        GPIO.output(self.MOTOR_B_A1, GPIO.LOW)
        GPIO.output(self.MOTOR_B_B1, GPIO.LOW)

    def clockwise(self):
        GPIO.output(self.MOTOR_B_A1, GPIO.HIGH)
        GPIO.output(self.MOTOR_B_B1, GPIO.LOW)

    def counterclockwise(self):
        GPIO.output(self.MOTOR_B_A1, GPIO.LOW)
        GPIO.output(self.MOTOR_B_B1, GPIO.HIGH)


dcmotor = DCMotor()

dcmotor.clockwise()
time.sleep(5)
dcmotor.stop()
time.sleep(1)
dcmotor.counterclockwise()
time.sleep(5)
dcmotor.stop()
