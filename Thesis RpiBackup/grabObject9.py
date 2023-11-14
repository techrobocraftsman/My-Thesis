import cv2
import numpy as np
import serial
import time
import functions as fnc
#[41,51,18, 94,206,255]
myColors = [
            [38,44,88, 64,252,255],#0 green bunny
            #[108,87,15,124,255,255],#1 blue marker
            [110,137,85,118,231,255],#1 blue marker
            [59,75,54,88,173,180],#2 green spray paint
            [17,152,137,25,231,255]#3 orange ball
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

static = 0
flag = 0
done=0
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
    amplify = 150
    
    wait = 10
    interval = 20
    t1 = wait+interval
    #print(t1,t2,t3)
    
    frameW = img.shape[1]
    frameH = img.shape[0]
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    count = 0
    newPoints=[]
    
    lower = np.array(color[0:3])
    upper = np.array(color[3:6])
    mask = cv2.inRange(imgHSV,lower,upper)
    #shMask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    #result = cv2.bitwise_and(img, img, mask=mask)
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
    global static,pos,flag,done
    inc = 1
    global rx,ry,rz,az,offset,grip,Max,Min
    
    #originX, originY = frameW//2, frameH//2
    originX, originY = 220, frameH-200
    mDir,tDir = 'F','L'
    wait = 20
    if static<=wait:
        if change:
            centX, centY = x+w//2,y+h//2
            dx, dy = centX-originX, originY-centY
            if dy > tolerance+5 :
                if ry<170:ry += inc
                else: speed = 20
            elif dy < -tolerance-5:
                if ry>Min[2]:ry -= inc
                else: speed = -20
            if abs(dx) > tolerance : 
                turn = int(dx*2*40/frameW)
            
            if abs(dx)<=tolerance and abs(dy)<=tolerance:
                static+=1
                print(static)
            else:static = 0
            
        else:
            centX, centY = originX, originY
        if speed<0:
            mDir = 'B'
            speed *= -1
        if turn<0:
            tDir = 'R'
            turn *= -1
        if 0<turn<7:
            turn=7
            #speed=5

        if static==0:Msg = f'{mDir}{speed} {tDir}{turn} y{ry} U{az} r{offset}'
        else: Msg = f'{mDir}{0} {tDir}{0} y{ry} U{az} r{offset}'
    
    elif static > wait:
        if az>-15 and flag==0:
            az-=5
            Msg = f'{mDir}{0} {tDir}{0} y{ry} U{az} r{offset}'
        elif az<=-15 and flag==0:
            flag=1
            time.sleep(1)
            
        if az<90 and flag==1:
            Msg = f'{mDir}{0} {tDir}{0} y{ry} U{az} g{offset}'
            az+=15
        elif az>=90 and flag==1:
            flag=2
            time.sleep(.5)
        
        if ry>70 and flag==2:
            ry-=10
            Msg = f'{mDir}{0} {tDir}{0} y{ry} U{az} g{offset}'
        elif ry<=70 and flag==2:
            flag=3
            time.sleep(.5)
        
        if flag==3:
            Msg = f'{mDir}{0} {tDir}{0} y{ry} U{az} r{offset}'
            done+=1
            print(done)
    #draw
    cv2.rectangle(imgResult, (x, y), (x+w, y+h), (0, 0, 255), 3)
    cv2.line(imgResult, (0,originY), (frameW, originY), (0,255,0), 1)
    cv2.line(imgResult, (originX,0), (originX, frameH), (0,255,0), 1)
    cv2.circle(imgResult, (centX, centY), 4, (0,0,0), cv2.FILLED)
    cv2.circle(imgResult, (centX, centY), 8, (255,255,0), 2)
    cv2.line(imgResult, (originX,originY), (centX, originY), (255,255,255), 4)
    cv2.arrowedLine(imgResult, (centX, originY), (centX, centY), (0,0,0), 2)
    #imgStack = fnc.stackImages(0.6,([shMask,img],[result,imgResult]))
    #cv2.imshow("Stacked image", imgStack)
    cv2.imshow("Ai vision colorDetection", imgResult)
    
    #cv2.waitKey(1)
    
    return Msg
    

frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)
cap.set(10,150)

comm = Comm('/dev/ttyACM0')

#ry,az,offset,grip,rz = 120,110,10,'r',0
ry,az,offset,grip,rz = 130,100,10,'r',0
Max = [0,10,225,180,115];
Min = [0,-10,130,-15,0];

#initMsg = f'F{0} R{0} x{rx} y{ry} U{az} r{offset}'
#print(Msg)

while True:    
    Msg = findColor(myColors[0])
    print(Msg)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if done>10:
        break
    comm.sendData(Msg)
    comm.readData()
    time.sleep(.1)
cap.release()
cv2.destroyAllWindows()


