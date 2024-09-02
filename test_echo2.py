#!/usr/bin/python

from gpiozero import Device, PhaseEnableMotor, Robot, PhaseEnableRobot, LED, Servo, AngularServo, Button, DistanceSensor
from gpiozero.tools import zip_values
from gpiozero.pins.pigpio import PiGPIOFactory  
from time import sleep

factory = PiGPIOFactory()
rightGrass = DistanceSensor(echo=12, trigger=16, max_distance=2.0,pin_factory=factory) #Right grass, normally False
rightCenterGrass = DistanceSensor(echo=20, trigger=21, max_distance=2.0,pin_factory=factory) #Right Center grass, normally False
leftGrass = DistanceSensor(echo=19, trigger=26, max_distance=2.0,pin_factory=factory) #Left Center grass, normally False
leftCenterGrass = DistanceSensor(echo=6, trigger=5, max_distance=2.0,pin_factory=factory) #Left grass, normally False

while True:
   l = str(round(leftGrass.distance * 100,1))
   sleep(0.05)
   lc = str(round(leftCenterGrass.distance * 100,1))
   sleep(0.05)
   rc = str(round(rightCenterGrass.distance * 100,1))
   sleep(0.05)
   r = str(round(rightGrass.distance * 100,1))
   sleep(0.05)
   print("left:",l,"leftCenter",lc,"rightCenter",rc,"right",r)
