#!/usr/bin/env python3

import Jetson.GPIO as GPIO
import time


class DCMotor():
    def __init__(self):
        self.MOTOR_B_A1 = int(17)
        self.MOTOR_B_B1 = int(18)

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

if __name__ == '__main__':
    motor = DCMotor()
    motor.counterclockwise()
    time.sleep(0.2)
    motor.stop()
    time.sleep(0.2)
    motor.clockwise()
    time.sleep(1)
    motor.stop()
    time.sleep(0.2)
    GPIO.cleanup()