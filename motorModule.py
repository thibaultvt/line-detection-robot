import RPi.GPIO as GPIO          
from time import sleep

#   Initialization
#   Assign the appropriate GPIO pins, yours might be different!
GPIO.setmode(GPIO.BCM)
in1 = 23
in2 = 24
in3 = 22
in4 = 27
en1 = 25
en2 = 17

GPIO.setmode(GPIO.BCM)

#Declaring first motor as output
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)

#Declaring second motor as output
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)

GPIO.setup(en1,GPIO.OUT)
GPIO.setup(en2,GPIO.OUT)

#   PWM(channel, frequency)
p1=GPIO.PWM(en1,1000)
p2=GPIO.PWM(en2,1000)

#   Makes the robot drive forward
GPIO.output(in1,GPIO.HIGH)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.HIGH)
GPIO.output(in4,GPIO.LOW)

#   Start the PWM with a duty cycle of 0
p1.start(0)
p2.start(0)

#   End of initialization

#   Changes the PWM signals on the basis of the given steering angle (0-180)

def steeringWheel(steeringAngle):
    #   steeringAngle between 0 and 85 => GO LEFT
    steer = 90
    constant = 70
    if 0 <= steeringAngle <= 85:
        p1.ChangeDutyCycle(constant)
        p2.ChangeDutyCycle(steer)
        print('going left')
    elif 95 <= steeringAngle <= 180:
        p1.ChangeDutyCycle(steer)
        p2.ChangeDutyCycle(constant)
        print('going right')
    else:
        print('going straight')
        p1.ChangeDutyCycle(constant)
        p2.ChangeDutyCycle(constant)


while(1):
    x = int(input("SteeringAngle: "))
    steeringWheel(x)

