#!/usr/bin/python

from gpiozero import DistanceSensor
from time import sleep

rightGrass = DistanceSensor(echo=12, trigger=16, max_distance=2.0) #Right grass, normally False
rightCenterGrass = DistanceSensor(echo=20, trigger=21, max_distance=2.0) #Right Center grass, normally False
leftGrass = DistanceSensor(echo=19, trigger=26, max_distance=2.0) #Left Center grass, normally False
leftCenterGrass = DistanceSensor(echo=6, trigger=5, max_distance=2.0) #Left grass, normally False

while True:
   l = str(round(leftGrass.distance * 100,1))
   sleep(0.1)
   lc = str(round(leftCenterGrass.distance * 100,1))
   sleep(0.1)
   rc = str(round(rightCenterGrass.distance * 100,1))
   sleep(0.1)
   r = str(round(rightGrass.distance * 100,1))
   sleep(0.1)
   print("left:",l,"leftCenter",lc,"rightCenter",rc,"right",r)
