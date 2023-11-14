import cv2
import numpy as np
import serial
import time
import functions as fnc

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

static = 0
def findColor(color):
    Msg =''
    success, img = cap.read()
    imgResult = img.copy()
    speed, turn = 0,0
    x,y,z = 0,30,145
    w, h = 0,0
    dx, dy = 0,0
    centX, centY = 0,0
    tolerance = 10
    amplify = 200
    
    wait = 10
    interval = 20
    t1 = wait+interval
    t2 = wait+interval*2
    t3 = wait+interval*3
    t4 = wait+interval*4
    #print(t1,t2,t3)
    
    frameW = img.shape[1]
    frameH = img.shape[0]
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    count = 0
    newPoints=[]
    
    lower = np.array(color[0:3])
    upper = np.array(color[3:6])
    mask = cv2.inRange(imgHSV,lower,upper)
    shMask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    result = cv2.bitwise_and(img, img, mask=mask)
#****Draw contours
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
    global static,pos
    inc = 2
    global rx,ry,rz,az,offset,grip,Max,Min
    
    #for laser
    #originX, originY = (frameW//2), frameH-150
    originX, originY = (frameW//2)-70, frameH-100
    
    if change:
        centX, centY = x+w//2,y+h//2
        dx, dy = centX-originX, originY-centY
        if abs(dy) > tolerance :
            speed = int(dy*amplify/frameH)
        if abs(dx) > tolerance : 
            turn = int(dx*2*110/frameW)
        
        if abs(dx)<tolerance and abs(dy)<tolerance:
            static+=1
            print(static)
        else:static = 0
        
    else:
        centX, centY = originX, originY
    mDir,tDir = 'F','L'
    if speed>100:speed=100
    elif speed<-100:speed=-100
    if speed<0:
        mDir = 'B'
        speed *= -3
    if turn<0:
        tDir = 'R'
        turn *= -1
    if 0<turn and turn<30:
        turn = 30
        
    if static==0:Msg = f'{mDir}{speed} {tDir}{turn}'
    else: Msg = f'{mDir}0 {tDir}0'
        
        
    #draw
    cv2.rectangle(imgResult, (x, y), (x+w, y+h), (0, 0, 255), 3)
    cv2.line(imgResult, (0,originY), (frameW, originY), (0,255,0), 1)
    cv2.line(imgResult, (originX,0), (originX, frameH), (0,255,0), 1)
    cv2.circle(imgResult, (centX, centY), 4, (0,0,0), cv2.FILLED)
    cv2.circle(imgResult, (centX, centY), 8, (255,255,0), 2)
    cv2.line(imgResult, (originX,originY), (centX, originY), (255,255,255), 4)
    cv2.arrowedLine(imgResult, (centX, originY), (centX, centY), (0,0,0), 2)
    imgStack = fnc.stackImages(0.6,([shMask,img],[result,imgResult]))
    cv2.imshow("Stacked image", imgStack)
    #cv2.imshow("Ai vision colorDetection", imgResult)
    
    cv2.waitKey(1)
    return Msg
    

frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)
cap.set(10,150)

myColors = [
            [38,44,88, 64,252,255],#0 green bunny
            #[108,87,15,124,255,255],#1 blue marker
            [110,137,85,118,231,255],#1 blue marker
            [59,75,54,88,173,180],#2 green spray paint
            [17,152,137,25,231,255],#3 orange ball
            #[109,70,100,118,190,245],#4 blue tape
            [98,50,32,126,235,140],#4 blue tape
            [27, 76,160,33,200,255],#5 yl tape
            [74,45,100,92,185,168], #6 green tape
            [52,51,169,70,245,255],  #7 Green Laser Light
            [50,42,160,75,137,255]   #8 Green Laser point light
            ]
ry,az,offset,grip,rz = 0,65,60,'r',85
Max = [0,10,225,180,115];
Min = [0,-10,30,-15,0];

comm = Comm('/dev/ttyACM0')
initMsg = f'F{0} R{0} y{ry} U{az} r{offset}'
comm.sendData(initMsg)
comm.readData()

while True:    
    Msg = findColor(myColors[0])
    
    print(Msg)
    comm.sendData(Msg)
    comm.readData()
cap.release()
cv2.destroyAllWindows()

