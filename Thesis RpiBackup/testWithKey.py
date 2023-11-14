import cv2
import numpy as np
import serial
import time
import keyboardModule as km


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


def test():
    global rx,ry,rz,az,offset,grip,Max,Min
    s,t = 'F0','L0'
    sp,tr = 50,50
    inc = 3
    if km.getKey('w'):
        s='F'+str(sp)
    elif km.getKey('s'):
        s='B'+str(sp)
    elif km.getKey('a'):
        t='L'+str(tr)
    elif km.getKey('d'):
        t='R'+str(tr)
    elif km.getKey('q'):
        t='Q'+str(tr)
    elif km.getKey('e'):
        t='E'+str(tr)
        
    elif km.getKey('UP'):
        if ry<Max[2]:ry+=inc
    elif km.getKey('DOWN'):
        if ry>Min[2]:ry-=inc
    elif km.getKey('PAGEUP'):
        if az<Max[3]:az+=inc
    elif km.getKey('PAGEDOWN'):
        if az>Min[3]:az-=inc
        
    elif km.getKey('KP8'):
        if offset<Max[4]:offset+=inc
    elif km.getKey('KP2'):
        if offset>Min[4]:offset-=inc
        
    elif km.getKey('RETURN'):
        grip='g'
    elif km.getKey('BACKSPACE'):
        grip='r'
        
    time.sleep(.05)
    aDir = 'U'
    if az<0:
        rz=-1*az 
        aDir = 'D'
    else: rz=az
    if offset<0:offset=0
    #Msg = f'{s} {t}'
    #Msg = f'x{rx} y{ry} {aDir}{rz} {grip}{offset}'
    Msg = f'{s} {t} y{ry} {grip}{offset} {aDir}{rz}'
    return Msg

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)
cap.set(10,150)

ry,az,offset,grip,rz = 0,65,60,'r',85
#ry,az,offset,grip,rz = 120,110,10,'r',0
# 170 140 10
Max = [0,10,225,180,115];
Min = [0,-10,0,-15,0];

comm = Comm('/dev/ttyACM0')
km.init()
while True:
    success,img = cap.read()
    Msg = test()
    
    print(Msg)
    comm.sendData(Msg)
    comm.readData()
    
    frameW = img.shape[1]
    frameH = img.shape[0]
    originX, originY = 160, frameH-100
    
    cv2.line(img, (0,originY-5), (frameW, originY-5), (0,255,0), 1)
    cv2.line(img, (originX,0), (originX, frameH), (0,255,0), 1)
    
    cv2.imshow("camera", img)
    cv2.waitKey(1)
    
cap.release()
cv2.destroyAllWindows()