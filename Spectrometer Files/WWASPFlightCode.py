#Flight Code v1.0, now working with Arduino
#Make sure the Firmata sketch is uploaded to the Arduino
##File > Examples > Firmata > StandardFirmata

#Code connects to the Arduino through serial connection

#imported packages
#import numpy as np #Might need numpy for calculations, especially base values or converting data. Not used in v1.0
import pyfirmata
import RPi.GPIO as gpio
import time
#import os #Might need os to set up file paths and the like, or at least working directories. Not used in v1.0
import serial
import cv2

#Bootup Process

#Setting constants to compare inputs to
ExpVolt = 0
ExpCur = 0
MaxTemp = 90

#Setting up Camera USB port number
CamPort = 0

#Setting up USB inputs. I think they need to be at 1200 baud, but I'm new to this package so it might be wrong.
ThermometerUSB = serial.Serial('/Thermo/Path/Here', 1200) #Setting up USB Thermometer port
ThermoInput = ThermometerUSB.read() #Setting up command to read thermometer data
SpectraUSB = serial.Serial('/Spectrometer/Path/Here', 1200) #Setting up USB port for spectrometer
SpectraInput = SpectraUSB.read() #Setting up command to read spectrometer input
EncoderUSB = serial.Serial('/Encoder/Path/Here', 1200)
EncoderInput = EncoderUSB.read()

#GPS and Time both from HASP
GPSstring = 0
Time = 0

#Connecting to Arduino board
board = pyfirmata.Arduino('TestName') #Establishing serial connection

#Setting an iterator to read statuses
it = pyfirmata.util.Iterator(board)
it.start()

#defining pin uses for stepper motor
StepPins = [1,2,3,4]
for pin in StepPins:
    gpio.setup(pin, gpio.out) #Setting all the Stepper Motor pins to outputs
    gpio.output(pin, 0) #Setting outputs to false for now, clean slate

StepSignal = board.get_pin('d:5:i') #Pin for the base position signal

#Pin sequence to turn the motor later
#Pin Sequence should funtion as a half step for a full rotation, I think
PinSeq = [
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1],
    [1,0,0,1]
]

#Checking Stepper Motor signal, attempting to move it
HomePosTries = 0
HomePosMaxTries = 365*2
HomePosError = 0
while StepSignal.read() is False:
    for step in range(7):
        for pin in [1,2,3,4]:
            gpio.output(StepPins[pin], PinSeq[step][pin])
        time.sleep(0.01)
    HomePosTries = HomePosTries + 1
    if HomePosTries == HomePosMaxTries:
        HomePosError = 1
        break


#Sending Housekeeping string to ground team
#CODE GOES HERE

#Starting main data recording loop
while HomePosError == 0:
    HASPtime = 0 #Placeholder value for now, needs to retrieve HASP time
    GPSstring = 0 #Placeholder value for now, needs to retrieve the GPS data from HASP
    spectradata = open("/path/to/file/spectra{0}.txt".format(HASPtime), 'a') #creating the spectra file
    auxdata = open("/path/to/file/aux{0}.txt".format(HASPtime), 'a') #Creating auxilary data file
    
    #Take Image code here. Should take image and rename it to path/to/file/image{HASPtime}.png . Not super certain about this code.
    CamImage = cv2.VideoCapture(CamPort)
    time.sleep(0.1) #Should give the camera a second to warm up, stop the image from being really dark
    return_value, image = CamImage.read() #Should actually take the image
    cv2.imwrite('image{0}.png'.format(HASPtime), image) #Should save the image with the file name
    
    #Writing temperature and GPS data to auxilary data sheet
    auxdata.write(ThermoInput)
    auxdata.write(GPSstring)
    
    HomePosTries = 0
    HomePosError = 0
    while StepSignal.read() is False:
        for step in range(7):
            for pin in StepPins:
                gpio.output(StepPins[pin], PinSeq[step][pin])
            time.sleep(0.01)
            HomePosTries = HomePosTries + 1
            if HomePosTries == HomePosMaxTries:
                HomePosError = 1
                break
    if HomePosError == 1:
        placeholder = 0
        
        #HASP Housekeeping string code goes here
    
    for moves in range(6):
        for turns in range(19):
            for step in range(7):
                for pin in StepPins:
                    gpio.output(StepPins[pin], PinSeq[step][pin])
                    time.sleeop(0.01)
            spectradata.write(SpectraInput) #Is this right?
            
            #Not sure how the encoder's readout will work, but it'll change how the following few lines function
            if EncoderInput != 10:
                EncoderError = 1
            if EncoderInput == 10:
                EncoderError = 0
            auxdata.write(EncoderError) #Writting the error value to auxilary data
            
            
else:
    while StepSignal.read() is False:
        for step in range(7):
            for pin in [1,2,3,4]:
                gpio.output(StepPins[pin], PinSeq[step][pin])
            time.sleep(0.01)
            HomePosTries = HomePosTries + 1
        if HomePosTries == HomePosMaxTries:
            HomePosError = 1
            #Request reboot line?
            break
    else:
        HomePosError = 0
        







