import Jetson.GPIO as GPIO

class DCMotor():
    def __init__(self) -> None:
        self.MOTOR_B_A1=8
        self.MOTOR_B_B1=10

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.MOTOR_B_A1,GPIO.OUT)
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