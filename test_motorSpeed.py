from gpiozero import Device, PhaseEnableMotor, Robot, PhaseEnableRobot, LED, Servo, AngularServo, Button, DistanceSensor
from gpiozero.tools import zip_values
from gpiozero.pins.pigpio import PiGPIOFactory  
import time

MotorLSpeedPin = Button(23); MotorLSpeed = 0
MotorRSpeedPin = Button(7); MotorRSpeed = 0
MotorLBackSpeedPin = Button(9); MotorLBackSpeed = 0
MotorRBackSpeedPin = Button(27); MotorRBackSpeed = 0

def MotorLSpeedCount():
  global MotorLSpeed, MotorLSpeedPin
  MotorLSpeed = MotorLSpeed + 1
def MotorRSpeedCount():
  global MotorRSpeed, MotorRSpeedPin
  MotorRSpeed = MotorRSpeed + 1
def MotorLBackSpeedCount():
  global MotorLBackSpeed, MotorLBackSpeedPin
  MotorLBackSpeed = MotorLBackSpeed + 1
def MotorRBackSpeedCount():
  global MotorRBackSpeed, MotorRBackSpeedPin
  MotorRBackSpeed = MotorRBackSpeed + 1
#create calls to interrupts
MotorLSpeedPin.when_pressed = MotorLSpeedCount
MotorRSpeedPin.when_pressed = MotorRSpeedCount
MotorLBackSpeedPin.when_pressed = MotorLBackSpeedCount
MotorRBackSpeedPin.when_pressed = MotorRBackSpeedCount

while True:
  print("MotorL",MotorLSpeed,"MotorR",MotorRSpeed,"MotorLBack",MotorLBackSpeed,"MotorRBack",MotorRBackSpeed)
  time.sleep(1)
