#Control for hoverboard function. riorand board. from: #https://mad-ee.com/controlling-a-hoverboard-motor-with-a-simple-arduino/
#run with python3 -m flask --app mower/mower_v1p1.py run --host=0.0.0.0
#1.2 - v2 printed control arms. balance forward w/ curve
#1.3 - forward balance wheels function in robot class, ratio
#1.4 - echo location sensors
#1.5 - 4 wheel drive

# import the necessary packages
from flask import Flask, render_template
import math
import numpy as np
import os
from glob import glob
import time
from datetime import datetime
from gpiozero import Device, PhaseEnableMotor, Robot, PhaseEnableRobot, LED, Servo, AngularServo, Button, DistanceSensor
from gpiozero.tools import zip_values
from gpiozero.pins.pigpio import PiGPIOFactory  
import py_qmc5883l
from threading import Thread

#TO DO: add object detect/emergency off. Add 2 motors. Add mower and test.
#TO DO: Speed up wheels as battery drains. Count time / wheel turns
#TO DO: auto throttle-off safety. emergency off switch for start/stop on mower.
autoMode=0 #manual mode to start
flask=1 #flask is turned on - so we always start website
speed=0.37 #this is how fast of max (1) we should go. Can adjust if too fast or slow with your wheels. Make programmable?
timeBetweenSensorReads=0.1 #time to wait between reading sensors
timeToFlushSensorHistory=0.4 #how long it will take to completely clear out the sensor array history (how fast the mower reacts to changes in terrain/grass line)
currentCurve_left = 0 #using this as a steering bias if not straight
currentCurve_right = 0
reactivity = 1 #0 to 1, shows how quickly to react to motors not turning the same speed
max_sensor_distance = 2.0
backWheelAdjust = 1.1

#define the wheels and brake pins
brake = LED(14)
brake.off() #this sets nobrake pin high which turns off brake
MotorL=PhaseEnableMotor(15,18) #dir,pwm
MotorLBack=PhaseEnableMotor(25,22)
MotorR=PhaseEnableMotor(11,10)
MotorRBack=PhaseEnableMotor(13,8)
MotorLSpeedPin = Button(23); MotorLSpeed = 0
MotorRSpeedPin = Button(24); MotorRSpeed = 0
MotorLBackSpeedPin = Button(27); MotorLBackSpeed = 0
MotorRBackSpeedPin = Button(7); MotorRBackSpeed = 0

#define the sensors
#opticalBump = Button(12,bounce_time=1) #normally False
leftBump = Button(17) #Left bumper, normally True
rightBump = Button(1) #Right bumper, normally True
rightGrass = DistanceSensor(echo=12, trigger=16, max_distance=max_sensor_distance) #Right grass, normally False
rightCenterGrass = DistanceSensor(echo=20, trigger=21, max_distance=max_sensor_distance) #Right Center grass, normally False
leftGrass = DistanceSensor(echo=19, trigger=26, max_distance=max_sensor_distance) #Left Center grass, normally False
leftCenterGrass = DistanceSensor(echo=6, trigger=5, max_distance=max_sensor_distance) #Left grass, normally False
currentTime=datetime.now()
#grassInterrupt = [[int(leftGrass.is_pressed),int(leftCenterGrass.is_pressed),int(rightCenterGrass.is_pressed),int(rightGrass.is_pressed)],[currentTime,currentTime,currentTime,currentTime]] #current settings

#define the compass.  Here's some notes about it. It's not HMC as advertised
#from https://forums.raspberrypi.com//viewtopic.php?f=28&t=172244 -- this is QMC not HMC, that's not made any more bottom of page get drivers: https://github.com/RigacciOrg/py-qmc5883l
#PINS: vcc (yellow) -> 3.3V pin1.  Gnd Green -> Gnd pin 9.  orange SCL -> GPIO3 (pin5). red SDA->GPIO2 (pin3). DRDY unused.
#Don't connect drdy to any wire, it caused xy axis to shift to yz axis. compass broke!
#Enable I2C: sudo raspi-config (activate it under interfaces->i2c)
#Check connected up: sudo i2cdetect -y 1  (see output) - 0d address
#cd ~/mower/; git clone https://github.com/RigacciOrg/py-qmc5883l
#add py-qmc5883l to python path variable
  #sudo vi /etc/profile
  #At the top add these two lines (adjust path if necessary):
  #export ADDPATH=/home/gibby/mower/py-qmc5883l
  #export PYTHONPATH=$PYTHONPATH:$ADDPATH
  #reboot
sensor = py_qmc5883l.QMC5883L()
m = sensor.get_magnet()
print(m)

#robot = PhaseEnableRobot(left=(15, 18), right=(11, 10), pwm=True, pin_factory=None)
  #PhaseEnableRobot only exists in >2.0 but it doesn't have curve left/right. Need 2.0 to declare phase enable motors in robot, then can curve
  #but rpi 3 and 02w don't have <2.0 so they're stuck w the definition above w/o curve. 
  #So I need to treat motors separately and make my own class of robot. 
  #I'll basically copy this: https://gpiozero.readthedocs.io/en/stable/_modules/gpiozero/boards.html#Robot (look at v1.6.2 also code)
  #background: https://gpiozero.readthedocs.io/en/latest/compat.html   
class mowerRobot:
  def __init__(self, leftMotor, rightMotor, leftBackMotor, rightBackMotor):
    #super().__init__(left_motor=leftMotor, right_motor=rightMotor)
    self.left_motor=leftMotor
    self.right_motor=rightMotor
    self.leftBack_motor=leftBackMotor
    self.rightBack_motor=rightBackMotor
  def forward(self, speed=1, *, curve_left=0, curve_right=0, previousCommand="straight", command="straight", intentionalAngle=False):
    global MotorRSpeed, MotorLSpeed, currentCurve_right, currentCurve_left
    #check if wheels are turning at the same speed. 
    if previousCommand == "straight" and command == "straight" and intentionalAngle == False:
      #we've been going straight for at least 2 commands, 45 count = 1 wheel rotation, adjust curve as needed
      #also intentionalAngle = False, in other words we're not intentionally angling to get back on line
      if MotorRSpeed > 15 or MotorLSpeed > 15:
        if MotorRSpeed < MotorLSpeed:
          #right side is turning slower. add to right speed or subtract from left
          adder = round(MotorLSpeed/MotorRSpeed * speed - speed,2) * reactivity
          if currentCurve_left > 0:
            currentCurve_left = round(currentCurve_left - adder,2)
            currentCurve_right = 0
          else:
            currentCurve_right = round(currentCurve_right + adder,2)
            currentCurve_left = 0
        if MotorLSpeed < MotorRSpeed:
          #left side is turning slower. add to left side speed or subtract from right
          adder = round(MotorRSpeed/MotorLSpeed * speed - speed,2) * reactivity
          if currentCurve_right > 0:
            currentCurve_right = round(currentCurve_right - adder,2)
            currentCurve_left = 0
          else:
            currentCurve_left = round(currentCurve_left + adder,2)
            currentCurve_right = 0
        if currentCurve_left < 0:
          currentCurve_left = 0 #we can get negatives if we subtract as such above, can't set curve negative
        if currentCurve_right < 0:
          currentCurve_right = 0
        print("Recalculating - WheelLCount:",MotorLSpeed,"WheelRCount:",MotorRSpeed,"currentCurve_left",currentCurve_left,"currentCurve_right",currentCurve_right)
        MotorRSpeed = 0; MotorLSpeed = 0; #zero out now that we've made it above 45 counts and adjusted
      #we've recalculated speed now if possible, lets actually move forward.
      if not 0 <= curve_left <= 1:
        raise ValueError('curve_left must be between 0 and 1')
      if not 0 <= curve_right <= 1:
        raise ValueError('curve_right must be between 0 and 1')
      if curve_left != 0 and curve_right != 0:
        raise ValueError("curve_left and curve_right can't be used at the same time")
      self.left_motor.forward((speed + currentCurve_left) * (1 - curve_left))
      self.right_motor.forward((speed + currentCurve_right) * (1 - curve_right))
      print("2Str: Forward Left:",str((speed + currentCurve_left) * (1 - curve_left)),"Forward Right:",str((speed + currentCurve_right) * (1 - curve_right)),"WheelLCount:",MotorLSpeed,"WheelRCount:",MotorRSpeed)
    elif previousCommand == "straight" and intentionalAngle == True:
      #we just started turning. Zero out stats on wheel and execute turn
      MotorRSpeed = 0; MotorLSpeed = 0;
      self.left_motor.forward((speed + currentCurve_left) * (1 - curve_left))
      self.right_motor.forward((speed + currentCurve_right) * (1 - curve_right))
      print("1Trn: Forward Left:",str((speed + currentCurve_left) * (1 - curve_left)),"Forward Right:",str((speed + currentCurve_right) * (1 - curve_right)),"WheelLCount:",MotorLSpeed,"WheelRCount:",MotorRSpeed)
    elif (previousCommand == "angleL" or previousCommand == "angleR") and intentionalAngle == True and MotorRSpeed + MotorLSpeed > 30:
      # we are intentionally turning and we're on our second or more command. Get those wheel counts working
      if command == "angleR":
        if MotorRSpeed < MotorLSpeed*.8:
          #left turning too fast, speed up right
          adder = round(((MotorLSpeed * .8)/MotorRSpeed) * speed - speed,2) * reactivity
          if currentCurve_left > 0:
            currentCurve_left = round(currentCurve_left - adder,2)
            currentCurve_right = 0
          else:
            currentCurve_right = round(currentCurve_right + adder,2)
            currentCurve_left = 0
        if MotorRSpeed > MotorLSpeed * .8:
          #r motor is turning too hard. speed up left
          adder = round((MotorRSpeed/(MotorLSpeed * .8)) * speed - speed,2) * reactivity
          if currentCurve_right > 0:
            currentCurve_right = round(currentCurve_right - adder,2)
            currentCurve_left = 0
          else:
            currentCurve_left = round(currentCurve_left + adder,2)
            currentCurve_right = 0
        if currentCurve_left < 0:
          currentCurve_left = 0 #we can get negatives if we subtract as such above, can't set curve negative
        if currentCurve_right < 0:
          currentCurve_right = 0  
      if command == "angleL":
        if MotorLSpeed < MotorRSpeed * .8:
         #right turning too fast, speed up left
          adder = round(((MotorRSpeed * .8)/MotorLSpeed) * speed - speed,2) * reactivity
          if currentCurve_right > 0:
            currentCurve_right = round(currentCurve_right - adder,2)
            currentCurve_left = 0
          else:
            currentCurve_left = round(currentCurve_left + adder,2)
            currentCurve_right = 0
        if MotorLSpeed > MotorRSpeed * .8:
          #l motor is turning too hard. speed up right
          adder = round((MotorLSpeed/(MotorRSpeed * .8)) * speed - speed,2) * reactivity
          if currentCurve_left > 0:
            currentCurve_left = round(currentCurve_left - adder,2)
            currentCurve_right = 0
          else:
            currentCurve_right = round(currentCurve_right + adder,2)
            currentCurve_left = 0
        if currentCurve_left < 0:
          currentCurve_left = 0 #we can get negatives if we subtract as such above, can't set curve negative
        if currentCurve_right < 0:
          currentCurve_right = 0
      self.left_motor.forward((speed + currentCurve_left) * (1 - curve_left))
      self.right_motor.forward((speed + currentCurve_right) * (1 - curve_right))
      print("Trn2: Forward Left:",str((speed + currentCurve_left) * (1 - curve_left)),"Forward Right:",str((speed + currentCurve_right) * (1 - curve_right)),"WheelLCount:",MotorLSpeed,"WheelRCount:",MotorRSpeed)
      if MotorRSpeed > 55 or MotorLSpeed > 55:
        MotorRSpeed = 0; MotorLSpeed = 0;
    else:
      #just moved from turning or backing or something else to straight. don't pay attention to wheel counts, instead 0 them out and start recounting
      MotorRSpeed = 0; MotorLSpeed = 0;
      '''#Then kick out the turn - ignore straight command and turn the other direction for a beat. Save previous command as straight anyway
      if previousCommand == "angleR" or previousCommand == "angleL" and intentionalAngle == False:
        #we're straight (only set intentionalAngle True when we're angling on purpose) and we were just going straight. Kick it out.
        if previousCommand == "angleR":
          #angle back left for a moment
          robot.left(speed)
        if previousCommand == "angleL":
          robot.right(speed)
        time.sleep(0.4)
        print("Not forward, kicking out turn instead")
      else:'''
      #Previous command was EOR or backup or intentional angle. Don't continue doing that. instead drive forward so we at least follow this command
      self.left_motor.forward((speed + currentCurve_left) * (1 - curve_left))
      self.right_motor.forward((speed + currentCurve_right) * (1 - curve_right))
      print("1Str: Forward Left:",str((speed + currentCurve_left) * (1 - curve_left)),"Forward Right:",str((speed + currentCurve_right) * (1 - curve_right)),", No wheel Count")
  def backward(self, speed=1, *, curve_left=0, curve_right=0):
    if not 0 <= curve_left <= 1:
      raise ValueError('curve_left must be between 0 and 1')
    if not 0 <= curve_right <= 1:
      raise ValueError('curve_right must be between 0 and 1')
    if curve_left != 0 and curve_right != 0:
      raise ValueError("curve_left and curve_right can't be used at the same time")
    self.left_motor.backward(speed * (1 - curve_left))
    self.right_motor.backward(speed * (1 - curve_right))
    self.leftBack_motor.backward(speed*backWheelAdjust * (1 - curve_left))
    self.rightBack_motor.backward(speed*backWheelAdjust * (1 - curve_right))
  def left(self, speed=1):
    self.left_motor.backward(speed)
    self.right_motor.forward(speed)
    self.leftBack_motor.backward(speed*backWheelAdjust)
    self.rightBack_motor.forward(speed*backWheelAdjust)
  def right(self, speed=1):
    self.left_motor.forward(speed)
    self.right_motor.backward(speed)
    self.leftBack_motor.forward(speed*backWheelAdjust)
    self.rightBack_motor.backward(speed*backWheelAdjust)
  def stop(self):
    self.left_motor.stop()
    self.right_motor.stop()
    self.leftBack_motor.stop()
    self.rightBack_motor.stop()
robot = mowerRobot(MotorL,MotorR,MotorLBack,MotorRBack)

#useful functions
def turnMower(direction,degrees):
  initialCompass=sensor.get_bearing()
  currentCompass=initialCompass
  if direction == "right":
    robot.right(speed); 
  else:
    robot.left(speed);
  while abs(initialCompass-currentCompass) % 360 < degrees or abs(initialCompass-currentCompass) % 360 > (360-degrees):
    time.sleep(0.02); 
    currentCompass=sensor.get_bearing()
  print("initial compass:",str(initialCompass),"currentCompass",str(currentCompass),"difference",str(abs(initialCompass-currentCompass)%360))
def sensorWeightedAverage(grassSensor):
  global grassSide
  numMeasurements=len(grassSensor) #number of measurements
  numSensors=len(grassSensor[0]) #number of sensors on mower
  #get the weights based on number of measurements
  linArr = np.arange(1, numMeasurements+1)
  arrSum = numMeasurements * (numMeasurements+1) // 2
  reverseWeights = linArr/arrSum
  #row by row add up the weighted values in columns
  weightedGrassSensor = [0] * numSensors
  for (i,row) in enumerate(grassSensor):
    for (j,value) in enumerate(row):
      if value == max_sensor_distance * 100:
        value = 2.0 #we only get max sensor distance when no echo comes back, because grass is pressed right up on sensor. Correct for this
      weightedGrassSensor[j]=weightedGrassSensor[j]+reverseWeights[numMeasurements-1-i]*value
  #sum up the values in columns - unused
  #sumGrassSensor=[sum(x) for x in zip(*grassSensor)]
  for i in range(numSensors):
    weightedGrassSensor[i]=round(weightedGrassSensor[i],2)
  return weightedGrassSensor

#interrupts wheels and obstacles.  Take over wheels to move around obstacle.
def opticalBumpPress(initiator):
  global grassSide, interrupt, brake, robot, leftBump, rightBump
  duration = 1 #number of seconds to drive sideways to avoid the obstacle
  print("Starting avoidance routine, bumped by:",initiator)
  if interrupt == 0:
    interrupt=1 #tells the robot to stop doing other stuff
    #First try to back up and ensure the beam break is undone. If it's not, just stop. Could be the operator wants to stop the robot
    print("avoid obstacle - #1 brake"); brake.on(); time.sleep(0.1); robot.stop(); brake.off(); #first, stop things
    print("2 back up"); robot.backward(speed); time.sleep(2); brake.on(); time.sleep(0.1); robot.stop(); brake.off(); #second back up
    if leftBump.is_pressed and rightBump.is_pressed:
      print("3 turn opposite ",grassSide)
      if grassSide == "left":
        turnMower("right",90)
      else:
        turnMower("left",90)
      brake.on(); time.sleep(0.1); robot.stop(); brake.off(); #third turn 90
      print("4 forward for ",str(duration)); robot.forward(speed); time.sleep(duration); brake.on(); time.sleep(0.1); robot.stop(); brake.off(); #fourth drive forward (around obstacle)
      print("5 turn ",grassSide)
      turnMower(grassSide,90)
      time.sleep(1); brake.on(); time.sleep(0.1); robot.stop(); brake.off(); #fifth turn 90 back to original direction 
      print("6 return control to main loop")
    else:
      print("can't reestablish the bumper connection after backup, quitting")
      autoMode = 0
      robot.stop()
  interrupt = 0
  return
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
#opticalBump.when_pressed = opticalBumpPress #activates "pressed" when object breaks beam
leftBump.when_released = opticalBumpPress #activates "released" when object depresses sensor bar
rightBump.when_released = opticalBumpPress #activates "released" when object depresses sensor bar
MotorLSpeedPin.when_pressed = MotorLSpeedCount
MotorRSpeedPin.when_pressed = MotorRSpeedCount
MotorLBackSpeedPin.when_pressed = MotorLBackSpeedCount
MotorRBackSpeedPin.when_pressed = MotorRBackSpeedCount

interrupt=0
grassSide="left"
#normal operation loop - keep grass sensors in "normal" operation with small turns
def autoRoutine():
  print("starting auto sequence")
  global interrupt, robot, brake, autoMode, MotorRSpeed, MotorLSpeed, currentCurve_left, currentCurve_right
  currentCurve_left = 0; currentCurve_right = 0; MotorRSpeed = 0; MotorLSpeed = 0
  robot.forward(speed) #get a real initial value to drive with
  time.sleep(0.1)
  #Assume mower is initially placed on a line - so 3 grass sensors are engaged and 1 is disengaged.  
  #Initially fill the queue, then add/pop as mower moves forward to get weighted average of where grass line is
  grassSensor=[[50,3,3,3] for i in range(int(timeToFlushSensorHistory/timeBetweenSensorReads))]
  #set the grass side based on initial grassSensor reading
  if grassSensor[0][0] > 10:
    grassSide="left"
  else:
    grassSide="right"

  unsupported = 0
  previousCommand = ""
  while autoMode == 1:
    #main loop where we mow grass. 
    
    #get new reading from grass sensors, discard oldest. Take average of all readings
    grassSensor.insert(0,[int(leftGrass.distance*100),int(leftCenterGrass.distance*100),int(rightCenterGrass.distance*100),int(rightGrass.distance*100)])
    #correct anomalies
    if grassSide == "left":
      #correct for anomolous reading of second sensor if outside one is in grass
      if grassSensor[0][0] < grassSensor[0][1]:
        grassSensor[0][1]=grassSensor[0][0]
    if grassSide == "right":
      if grassSensor[0][3] < grassSensor[0][2]:
        grassSensor[0][2]=grassSensor[0][3]
 
    grassSensor.pop()
    #weighted average of the readings - weight more recent readings more
    grassSensorAvg=sensorWeightedAverage(grassSensor) #this one used if we poll
    #grassSensorAvg=grassInterruptCheck() #use this for interrupt driven
    thresh=10 #what weighted average shoudl we count as a 1 vs a 0
    command="" #initially blank, only set if we find a "known" weighted set of sensor values

    if grassSensorAvg[0]>thresh and grassSensorAvg[1]<thresh and grassSide == "left":
      #0,1, left
      command="straight"
    elif grassSensorAvg[0]>thresh and grassSensorAvg[1]>thresh and grassSide == "left":
      #0,0, left
      command="angleR"
    elif grassSensorAvg[0]<thresh and grassSensorAvg[1]<thresh and grassSide == "left":
      #1,1, left
      command="angleL"
    if grassSensorAvg[2]<thresh and grassSensorAvg[3]>thresh and grassSide == "right":
      #1,0, right
      command="straight"
    elif grassSensorAvg[2]>thresh and grassSensorAvg[3]>thresh and grassSide == "right":
      #0,0, right
      command="angleL"
    elif grassSensorAvg[2]<thresh and grassSensorAvg[3]<thresh and grassSide == "right":
      #1,1, right
      command="angleR"

    if previousCommand == "angleR" and command == "angleL":
      command = "straight" #straighten out for one command
    if previousCommand == "angleL" and command == "angleR":
      command = "straight"

    if grassSensorAvg[0]>thresh and grassSensorAvg[1]>thresh and grassSensorAvg[2]>thresh and grassSensorAvg[3]>thresh:
      #0,0,0,0 - no grass, end of row, make 90's until we find grass OR quit after 2 90's - no grass
      command="EOR"
      #command="stop"

    if command != "":
      unsupported = 0 #we have a real command, so it's a supported case, zero that one out
    if unsupported > 50:
      command="stop" #we don't know what's happening, stop wheels and wait
    
    print("grassSensorAvg:",grassSensorAvg,"current:",grassSensor[0],"command:",command,"grassSide:",grassSide,"currentCurve_left:",currentCurve_left,"currentCurve_right:",currentCurve_right,"unsupported Count:",unsupported,"MotorLSpeed",MotorLSpeed,"MotorRSpeed",MotorRSpeed)
    #print(grassInterrupt)
    #Must obey interrupts - don't take over robot while interrupt is running
    #unset turn debt after return to straight after it built up
    #keep track of curve left or right (not both) that has built up to keep robot going on line
    if interrupt == 0:
      if command=="straight":
        robot.forward(speed,previousCommand=previousCommand,command=command)
      if command=="angleR":
        #turn, add a small portion to currentCurve. zero out other. over time we'll build up curve to keep us "normal" if needed
        robot.forward(speed,curve_right=0.2,previousCommand=previousCommand,command=command,intentionalAngle=True)
      if command=="angleL":
        #turn, add a small portion to currentCurve. zero out other. over time we'll build up curve to keep us "normal" if needed
        robot.forward(speed,curve_left=0.2,previousCommand=previousCommand,command=command,intentionalAngle=True)
      if command=="EOR":
        # End of row, no grass detected.
        # Back up a bit, turn 90 towards grass line. Drive forward to find new line. if yes any 1's, go on normally
        #  if can't find line (all grass), turn 90 toward grass again. = 180 turn, switch line side
        #  if can't find line (no grass), end.
        EORLimit = 8
        EORCount = 0
        while EORCount < EORLimit:
          robot.backward(speed); time.sleep(0.75)
          if grassSide == "left":
            turnMower("right",20)
          else:
            turnMower("left",20)
          robot.forward(speed); time.sleep(0.05)
          #check current sensor values - refill sensor values after turn
          for i in range (0,12):
            grassSensor.insert(0,[int(leftGrass.distance*100),int(leftCenterGrass.distance*100),int(rightCenterGrass.distance*100),int(rightGrass.distance*100)]); 
            grassSensor.pop(); 
            time.sleep(0.05);
          grassSensorAvg=sensorWeightedAverage(grassSensor)
          if grassSensorAvg[0] > thresh and grassSensorAvg[1] > thresh and grassSensorAvg[2] > thresh and grassSensorAvg[3] > thresh:
            #No Grass, go again
            EORCount = EORCount + 1
          else:
            #found some grass, end the loop and drive forward
            EORLimit = -1
        if EORCount == EORLimit:
          command="stop"
        else:
          #some grass was found, keep going forward and reading values
          robot.forward(speed)
      if command == "stop":
        #made the turns at the end of the row, no grass found. stop moving.
        robot.stop()
        print("stopping")
        autoMode = 0
        #TO DO: turn off throttle of engine
      time.sleep(timeBetweenSensorReads) #give just a bit of break between sensor readings
    previousCommand = command #save last one so we can tell when we change from turn to straight (to zero out wheel counts)
  print("done with auto mode, something set autoMode = 0");
  robot.stop()

#Flask setup and links to manual control
app = Flask(__name__)
@app.route("/")
def getPage():
  templateData = {
    'title' : 'NathanBuildsDIY robot mower v2!'
  }
  return render_template('index.html', **templateData)
@app.route("/fwd", methods=['GET', 'POST'])
def forward():
  print("forward"); robot.forward(speed)
  return ('', 204)
@app.route("/back", methods=['GET', 'POST'])
def back():
  print("back"); robot.backward(speed)
  return ('', 204)
@app.route("/right", methods=['GET', 'POST'])
def turn_right():
  print("right"); robot.right(speed)
  return ('', 204)
@app.route("/left", methods=['GET', 'POST'])
def turn_left():
  print("left"); robot.left(speed)
  return ('', 204)
@app.route("/stop", methods=['GET', 'POST'])
def stop():
  print("stopping"); brake.on(); robot.stop(); brake.off(); 
  return ('', 204)
@app.route("/auto", methods=['GET', 'POST'])
def auto():
  print("Calling auto routine"); 
  global autoMode
  autoMode = 1
  thread = Thread(target = autoRoutine, args = ()) #will stop when auto mode completes (stop command sets autoMode = 0) or when autoMode is set to 0 by clicking "manual" in website
  thread.start()
  return ('', 204)
@app.route("/stopauto", methods=['GET', 'POST'])
def stopauto():
  global autoMode
  print("setting Manual (autoMode=0)"); 
  autoMode = 0 #this will cause the auto thread to terminate
  return ('', 204)

if __name__ == "__main__":
  app.run(host='0.0.0.0', port=80, debug=True)
