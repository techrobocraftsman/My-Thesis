import cv2
import numpy as np

import serial
import time
#import keyboardModule as km

frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)
cap.set(10,150)
#[41,51,18, 94,206,255]
myColors = [
            [38,44,88, 64,252,255],#0 green bunny
            [108,87,15,124,255,255],#1 blue marker
            [17,152,137,25,231,255]#2 orange ball
            ]

class Comm:
    def __init__(self, port):
        self.port = port
        self.ser = serial.Serial(self.port, 9600, timeout=1)
        self.ser.reset_input_buffer()
    def enterData(self, val=''):
        if val == '':
            cmd = input("Enter command : ")
        else:
            cmd = val
        self.sendData(cmd)

    def sendData(self, cmd):
        #print(cmd)
        self.ser.write(cmd.encode())

    def readData(self):
        #while self.ser.inWaiting() == 0: pass
        if self.ser.inWaiting() > 0:
            answer = self.ser.readline()
            print(answer)
            self.ser.flushInput()
            
def findColor(img,color):
    speed, turn = 0,0
    x, y, w, h = 0,0,0,0
    dx, dy = 0,0
    centX, centY = 0,0
    tolerance = 10
    minVal = 10
    amplify = 80
    
    frameW = img.shape[1]
    frameH = img.shape[0]
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    count = 0
    originX, originY = frameW//2, frameH-100
    newPoints=[]
    
    lower = np.array(color[0:3])
    upper = np.array(color[3:6])
    mask = cv2.inRange(imgHSV,lower,upper)
    
#****	Draw contours
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    change = False
    #print(hierarchy)
    for cnt in contours:
        area = cv2.contourArea(cnt)     # area of the contour
        if area > 500:
            #print('Area = ' + str(area))
            cv2.drawContours(imgResult, cnt, -1, (255, 0, 0), 3)
            peri = cv2.arcLength(cnt, True)
            # print(peri)         # perimeter of the contour
            approxCor = cv2.approxPolyDP(cnt, 0.02*peri, True)
            #print(len(approxCor))
            objCor = len(approxCor)
            x, y, w, h = cv2.boundingRect(approxCor)
            change = True
            
    if change:
        centX, centY = x+w//2,y+h//2
        dx, dy = centX-originX, originY-centY
        if abs(dy) > tolerance :
            speed = int(dy*amplify/frameH)
        if abs(dx) > tolerance : 
            turn = int(dx*2*amplify/frameW)
    else:
        centX, centY = originX, originY
    mDir,tDir = 'F','L'
    if speed<0:
        mDir = 'B'
        speed *= -5
    if turn<0:
        tDir = 'R'
        turn *= -1
    
    
    Msg = mDir + str(speed) + ' ' + tDir + str(turn)
    #print(Msg)
    
    comm.sendData(Msg)
    comm.readData()
    #print(dx,dy,centX,centY,speed,turn)
    #print(speed,turn)
    
    #draw
    
    cv2.rectangle(imgResult, (x, y), (x+w, y+h), (0, 0, 255), 3)
    cv2.line(imgResult, (0,originY), (frameW, originY), (0,255,0), 1)
    cv2.line(imgResult, (originX,0), (originX, frameH), (0,255,0), 1)
    cv2.circle(imgResult, (centX, centY), 4, (0,0,0), cv2.FILLED)
    cv2.circle(imgResult, (centX, centY), 8, (255,255,0), 2)
    
    cv2.line(imgResult, (originX,originY), (centX, originY), (255,255,255), 4)
    cv2.arrowedLine(imgResult, (centX, originY), (centX, centY), (0,0,0), 2)
   
    
    #return speed,turn

while True:    
    comm = Comm('/dev/ttyACM0')
    success, img = cap.read()
    imgResult = img.copy()
    findColor(img, myColors[0])        
    cv2.imshow("Ai vision colorDetection", imgResult)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()













