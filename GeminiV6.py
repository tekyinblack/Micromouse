# Micromouse Gemini Robot
# code adapted from original examples by UKMARS.org
# v1 260228 first versio of class
# V2 260301 moved more line following specifics to class
# v3 260301 moved key line following routines
# v4 260405 added wallfollower routines, not complete baselined for safety
# v5 260405b new class library for micromouse robots, adopts newEncoders and ws2812b
# v6 260411 change markers to timings
#           add map functions
from machine import Pin,ADC,PWM,UART
import time
import os
from newEncoders import Encoders
from ws2812 import ws2812b

class Gemini ():
    # base class for gemini robot with Pico mezannine board
    # it is primarily about motors, switches and indicator leds
    def __init__(self,baseclass = "Gemini"):
        
        if baseclass == "Gemini":
            self.GeminiInit()
        elif baseclass == "Pico2Zero":
            self.Pico2ZeroInit()
            
        self.leftCount = 0         
        self.rightCount = 0
        self.robotState = 0
        self.motorSpeed = 0
        self.lastTime = time.ticks_ms()
        self.rightRev.duty_u16(65535)
        self.rightFwd.duty_u16(65535)
        self.leftRev.duty_u16(65535)
        self.leftFwd.duty_u16(65535)

    def GeminiInit(self):
        self.onBoardLED = Pin("LED", Pin.OUT)

        self.leftMezzLED = Pin(12,Pin.OUT)
        self.rightMezzLED = Pin(13,Pin.OUT)

        self.leftRev = PWM(Pin(3))
        self.leftRev.freq(2000)
        self.leftFwd = PWM(Pin(2))
        self.leftFwd.freq(2000)
        self.rightRev = PWM(Pin(5))
        self.rightRev.freq(2000)
        self.rightFwd = PWM(Pin(4))
        self.rightFwd.freq(2000)
        self.leftButton = Pin(15, Pin.IN, Pin.PULL_UP)
        self.rightButton = Pin(14, Pin.IN, Pin.PULL_UP)
        self.stopMotors()
        #self.encoders = Encoders()
        self.encoders = Encoders(0,8,9,1,6,7)
        
        
    
    def Pico2ZeroInit(self):
        # pin assignment for drag racing line follower
        # this class to be used for line following, drag racing and pursuit
        #self.onBoardLED = ws2812b(16,4) # note uses pio4 to avoid encoder pio's
        self.onBoardLED = Pin(11, Pin.OUT)
        self.leftMezzLED = Pin(12,Pin.OUT)
        self.rightMezzLED = Pin(13,Pin.OUT)
        self.leftRev = PWM(Pin(5))
        self.leftRev.freq(2000)
        self.leftFwd = PWM(Pin(6))
        self.leftFwd.freq(2000)
        self.rightRev = PWM(Pin(8))
        self.rightRev.freq(2000)
        self.rightFwd = PWM(Pin(7))
        self.rightFwd.freq(2000)
        self.leftButton = Pin(14, Pin.IN, Pin.PULL_UP)
        self.rightButton = Pin(4, Pin.IN, Pin.PULL_UP)
        self.stopMotors()
        #                pio num1, pin 1, pin 2 pio num2, pin 3, pin 4
        self.encoders = Encoders(0,0,1,1,2,3)
 

#Set the motor pwm duty cycle from required speed expressed as a percentage 0-100
    def leftMotor(self,speed):
        if speed < 0:
            speed = -speed
            if speed > 100:
                speed = 100
            dutyCycle = speed * 655
            self.leftFwd.duty_u16(65535 - int(dutyCycle))
            self.leftRev.duty_u16(65535)
        else:
            if speed > 100:
                speed = 100
            dutyCycle = speed * 655
            self.leftRev.duty_u16(65535 - int(dutyCycle))
            self.leftFwd.duty_u16(65535)

    def rightMotor(self,speed):
        if speed < 0:
            speed = -speed
            if speed > 100:
                speed = 100
            dutyCycle = speed * 655
            self.rightFwd.duty_u16(65535 - int(dutyCycle))
            self.rightRev.duty_u16(65535)
        else:
            if speed > 100:
                speed = 100
            dutyCycle = speed * 655
            self.rightRev.duty_u16(65535 - int(dutyCycle))
            self.rightFwd.duty_u16(65535)

    def stopMotors(self):       
        self.rightRev.duty_u16(65535)
        self.rightFwd.duty_u16(65535)
        self.leftRev.duty_u16(65535)
        self.leftFwd.duty_u16(65535)
        
    def getEncoders(self,reset = False):
        self.leftCount, self.rightcount = self.encoders.get_counts(reset)
        if reset:
            self.lastTime = time.ticks_ms()
        else:    
            current = time.ticks_ms()
            diff = self.lastTime - current
            if diff > 0:
                self.leftMotorSpeed = self.leftCount * 1000 / diff
                self.rightMotorSpeed = self.rightCount * 1000 / diff
            self.lastTime = current
        return self.leftCount, self.rightcount
    
    def calibrateMotors(self,speed):
        self.stopmotors()
        time.sleep(1)      
        self.rightMotor(25)
        self.leftMotor(25)
        time.sleep(1)
        tempLeftCount,tempRightCount = getEncoders(True)
        time.sleep(1)
        tempLeftCount,tempRightCount = getEncoders(False)
        leftMotorSpeed25 = self.leftMotorSpeed
        rightMotorSpeed25 = self.rightMotorSpeed
        self.rightMotor(50)
        self.leftMotor(50)
        time.sleep(1)
        tempLeftCount1,tempRightCount1 = getEncoders(False)
        leftMotorSpeed50 = self.leftMotorSpeed
        rightMotorSpeed50 = self.rightMotorSpeed
        self.stopmotors()
        
class GeminiLineFollower(Gemini):
    def __init__(self, baseclass = "Gemini"):
        super().__init__(baseclass)
        if baseclass == "Gemini":
            self.leftsensor  = ADC(28)
            self.rightsensor = ADC(26)
            self.radiussensor = ADC(27)
            self.startsensor = ADC(27)
            self.emitter = Pin(22,Pin.OUT)
            self.radiusEmitter = Pin(18,Pin.OUT)
            self.sensorScale = 1
        elif baseclass == "Pico2Zero":
            self.leftsensor  = ADC(28)
            self.rightsensor = ADC(27)
            self.radiussensor = ADC(29)
            self.startsensor = ADC(26)
            self.emitter = Pin(15,Pin.OUT)
            self.radiusEmitter = self.emitter
            self.sensorScale = 1
            
        self.leftSensorLED = Pin(21,Pin.OUT)
        self.centreSensorLED = Pin(20,Pin.OUT)
        self.rightSensorLED = Pin(19,Pin.OUT)

        
        self.leftSensorUnlit = 0
        self.rightSensorUnlit = 0
        self.radiusSensorUnlit = 0
        self.startSensorUnlit =0
        self.leftSensorValue = 0
        self.rightSensorValue = 0
        self.radiusSensorValue = 0
        self.startSensorValue =0
        
        #--------------------------------------------
        #calibration caps and collars
        self.leftMax = 0
        self.leftMin = 65535
        self.rightMax = 0
        self.rightMin = 65535
        self.radiusMax = 0
        self.radiusMin = 65535
        self.startMax = 0
        self.startMin = 65535
        
        self.radiusLeadingEdge = 0
        self.radiusTrailingEdge = 0
        self.startLeadingEdge = 0
        self.startTrailingEdge = 0
        
        #global flags for marker detection
        self.sfLeadingEdgeDetected = 0
        self.sfTrailingEdgeDetected = 0
        self.sfTrigger = 0
        self.radiusLeadingEdgeDetected = 0
        self.radiusTrailingEdgeDetected = 0
        self.radiusTrigger = 0
        self.crossoverDetected = 0
        
        
    def readSensors(self):
        #Values are derived by subtracting the lit value of a sensor from the unlit value 
        #Unlit raw readings close to 65000 indicate good separation from ambient light
        #The radius and start/finish sensors are multiplexed onto the same ADC channel using separate emitters
        #The UKMARS line follower sensor board gives high value readings for low incident light and low value readings for high incident light

        self.emitter.value(0)
        self.radiusEmitter.value(0)
        self.leftSensorUnlit = self.leftsensor.read_u16()
        self.rightSensorUnlit = self.rightsensor.read_u16()
        self.startSensorUnlit = self.startsensor.read_u16()
        self.radiusSensorUnlit = self.radiussensor.read_u16()
        
        self.emitter.value(1)
        time.sleep_us(15)
        leftSensorLit = self.leftsensor.read_u16()
        rightSensorLit = self.rightsensor.read_u16()
        startSensorLit = self.startsensor.read_u16()
        self.emitter.value(0)
        
        self.radiusEmitter.value(1)
        time.sleep_us(15)
        radiusSensorLit = self.radiussensor.read_u16()
        self.radiusEmitter.value(0)

        self.leftSensorValue = (self.leftSensorUnlit - leftSensorLit)
        self.rightSensorValue = (self.rightSensorUnlit - rightSensorLit) 
        self.startSensorValue = (self.startSensorUnlit - startSensorLit) 
        self.radiusSensorValue = (self.radiusSensorUnlit - radiusSensorLit) 
        
    #update the maximum and minimum reflected values
    def calibrateSensors(self):

        if self.leftSensorValue > self.leftMax: self.leftMax = self.leftSensorValue
        if self.leftSensorValue < self.leftMin: self.leftMin = self.leftSensorValue
        if self.rightSensorValue > self.rightMax: self.rightMax = self.rightSensorValue
        if self.rightSensorValue < self.rightMin: self.rightMin = self.rightSensorValue
        if self.radiusSensorValue > self.radiusMax: self.radiusMax = self.radiusSensorValue
        if self.radiusSensorValue < self.radiusMin: self.radiusMin = self.radiusSensorValue
        if self.startSensorValue > self.startMax: self.startMax = self.startSensorValue
        if self.startSensorValue < self.startMin: self.startMin = self.startSensorValue



    def checkMarkers(self):
        timer = time.ticks_ms()
        if (self.sfLeadingEdgeDetected):                                     # already detected leading edge of start/finish marker
            if(self.radiusLeadingEdgeDetected):
                self.crossoverDetected = timer
            if (self.startSensorValue < self.startTrailingEdge):                  # trailing edge detected
                self.sfTrailingEdgeDetected = timer
                if (self.crossoverDetected):                                 # ignore crossovers
                    self.sfLeadingEdgeDetected = 0
                    self.sfTrailingEdgeDetected = 0
                    self.sfTrigger = 0
                    if (not self.radiusLeadingEdgeDetected):                 # if the radius detection has been cleared clear the crossover flag
                        self.crossoverDetected = 0                       # otherwise leave it set to allow radius to ignore crossover and clear 
                else:
                    self.sfTrigger = timer                                    # trailing edge detected and no crossover 
                    self.sfLeadingEdgeDetected = 0
                    self.sfTrailingEdgeDetected = 0
            else:
                pass                                                    #Leading edge detected and sensor above trailing edge threshold so do nothing
        else:                                               # No SF leading edge previously detected so check for leading edge
            if (self.startSensorValue > self.startLeadingEdge):
                self.sfLeadingEdgeDetected = timer
                if (self.radiusLeadingEdgeDetected):
                    self.crossoverDetected = timer                           # radius leading edge also detected so must be a crossover
            else:                                           #No leading edge detected so do nothing
                pass

        if (self.radiusLeadingEdgeDetected):                                 # already detected leading edge of radius marker
            if(self.sfLeadingEdgeDetected):
                self.crossoverDetected = timer
            if (self.radiusSensorValue < self.radiusTrailingEdge):                # trailing edge detected
                self.radiusTrailingEdgeDetected = timer
                if (self.crossoverDetected):
                    self.radiusLeadingEdgeDetected = 0
                    self.radiusTrailingEdgeDetected = 0
                    self.radiusTrigger = 0
                    if (not self.sfLeadingEdgeDetected):                     # if the start finish detection has been cleared clear the crossover flag
                        self.crossoverDetected = 0                       # otherwise leave it set to allow start finish to ignore crossover and clear
                else:
                    self.radiusTrigger = timer                                # trailing edge detected and no crossover so register radius
                    self.radiusLeadingEdgeDetected = 0
                    radiusTrailingEdgeDetected = 0
            else:                                                       #Leading edge detected and sensor above trailing edge threshold so do nothing
                pass
        else:                                               #No Radius leading edge already detected so check for leading edge
            if (self.radiusSensorValue > self.radiusLeadingEdge):
                self.radiusLeadingEdgeDetected = timer
                if (self.sfLeadingEdgeDetected):
                    self.crossoverDetected = timer                           # s/f leading edge also detected so must be a crossover

    def storeStartRadius(self):
        self.radiusLeadingEdge = self.radiusMin + ((self.radiusMax-self.radiusMin)/2)   #Marker leading and trailing edge thresholds for marker detection
        self.radiusTrailingEdge = self.radiusMin + ((self.radiusMax-self.radiusMin)/3)  #with hysteresis
        self.startLeadingEdge = self.startMin + ((self.startMax-self.startMin)/2)
        self.startTrailingEdge = self.startMin + ((self.startMax-self.startMin)/3)

class GeminiWallFollower(Gemini):
    def __init__(self,baseclass = "Gemini"):
        super().__init__(baseclass)
        if baseclass == "Gemini":
            # These are the pin connection settings for use with the wall sensor board
            # phototransistor sensor pins
            self.leftSensor = ADC(28) # input from the left wall sensor
            self.rightSensor = ADC(26) # input from the right wall sensor
            self.frontSensor = ADC(27) # input from the front wall sensor
            #Triggers for LEDs
            self.sidesEmitter = Pin(22,Pin.OUT) # switches on the 2 side facing wall illumination LEDs
            self.frontEmitter = Pin(21,Pin.OUT) # switches on the forward facing wall illumination LEDs
            # These are the indicator LEDs on the wall sensor board
            self.leftSensorLED = Pin(20,Pin.OUT) # indicator LED for when left wall seen
            self.centreSensorLED = Pin(19,Pin.OUT) # indicator LED for when front wall seen
            self.rightSensorLED = Pin(18,Pin.OUT) # indicator LED for when right wall seen
        
        # Variables
        self.leftval = 2000
        self.rightval = 2000
        self.frontval = 5000 # <<< reduced from 6000 as robot more responsive
        
        self.leftWall = False
        self.rightWall = False
        self.frontWall = False
        
        self.leftSensorUnlit = 0
        self.rightSensorUnlit = 0
        self.leftSensorLit = 0
        self.rightSensorLit = 0
        self.frontSensorUnlit = 0
        self.frontSensorLit = 0
        self.leftSensorValue = 0
        self.rightSensorValue = 0
        self.frontSensorValue = 0
        
    def readSensors(self):
    #Values are derived by subtracting the lit value of a sensor from the unlit value 
    #The UKMARS wall sensor board gives high value readings for low incident light and low value readings for high incident light 
#     global leftSensorValue, rightSensorValue, frontSensorValue
#     global leftSensorLit, rightSensorLit, frontSensorLit
#     global leftSensorUnlit, rightSensorUnlit, frontSensorUnlit
#     global leftval, rightval, frontval

        self.leftSensorUnlit = self.leftSensor.read_u16()
        self.rightSensorUnlit = self.rightSensor.read_u16()
        self.sidesEmitter.value(1)
        time.sleep_us(75)
        self.leftSensorLit = self.leftSensor.read_u16()
        self.rightSensorLit = self.rightSensor.read_u16()
        self.sidesEmitter.value(0)
        
        self.frontSensorUnlit = self.frontSensor.read_u16()    
        self.frontEmitter.value(1)
        time.sleep_us(75)
        self.frontSensorLit = self.frontSensor.read_u16()
        self.frontEmitter.value(0)
        time.sleep_us(75)

        self.leftSensorValue = (self.leftSensorLit - self.leftSensorUnlit) # 
        self.rightSensorValue = (self.rightSensorLit- self.rightSensorUnlit) # 
        self.frontSensorValue = (self.frontSensorLit- self.frontSensorUnlit)
        
        if self.leftSensorValue > self.leftval:
            self.leftSensorLED.value(1)
        else:
            self.leftSensorLED.value(0)
        if self.rightSensorValue > self.rightval:
            self.rightSensorLED.value(1)
        else:
            self.rightSensorLED.value(0)   
        if self.frontSensorValue > self.frontval:
            self.centreSensorLED.value(1)
        else:
            self.centreSensorLED.value(0)
        
    def checkwalls(self):
#         global leftSensorValue, rightSensorValue, frontSensorValue
#         global leftWall, rightWall,frontWall, leftval, rightval,frontval
        if self.leftSensorValue > self.leftval:
            self.leftWall = True
        else:
            self.leftWall = False
        if self.rightSensorValue > self.rightval:
            self.rightWall = True
        else:
            self.rightWall = False
        if self.frontSensorValue > self.frontval:
            self.frontWall = True
        else:
            self.frontWall = False
