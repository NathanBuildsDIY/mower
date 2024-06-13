from gpiozero import Device, PhaseEnableMotor, Robot, PhaseEnableRobot, LED, Servo, AngularServo, Button, DistanceSensor
from gpiozero.tools import zip_values
from gpiozero.pins.pigpio import PiGPIOFactory  
import time

MotorL=PhaseEnableMotor(15,18) #dir,pwm
MotorLBack=PhaseEnableMotor(25,22)
MotorR=PhaseEnableMotor(11,10)
MotorRBack=PhaseEnableMotor(13,8)
MotorLSpeedPin = Button(23); MotorLSpeed = 0
MotorRSpeedPin = Button(24); MotorRSpeed = 0
MotorLBackSpeedPin = Button(27); MotorLBackSpeed = 0
MotorRBackSpeedPin = Button(7); MotorRBackSpeed = 0

while True:
  MotorR.forward(.5); MotorL.forward(.5); MotorRBack.forward(.5); MotorLBack.forward(.5)
  time.sleep(1)
  MotorR.stop(); MotorL.stop(); MotorRBack.stop(); MotorLBack.stop()
  time.sleep(1)
  MotorR.backward(.5); MotorL.backward(.5); MotorRBack.backward(.5); MotorLBack.backward(.5)
  time.sleep(1)
  MotorR.stop(); MotorL.stop(); MotorRBack.stop(); MotorLBack.stop()
  time.sleep(1)
