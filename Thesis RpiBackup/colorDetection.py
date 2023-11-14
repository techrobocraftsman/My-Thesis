import cv2
import numpy as np
import functions as fnc

frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)
cap.set(10,150)

while True:
    timer = cv2.getTickCount()
    
    success, img = cap.read()
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # ********** Masking The Image
    lower = np.array([53, 52, 8])
    upper = np.array([100, 229, 194])
    mask = cv2.inRange(imgHsv, lower, upper)
    result = cv2.bitwise_and(img, img, mask=mask)
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    #********** FPS calculations
    fps = cv2.getTickFrequency()/(cv2.getTickCount()-timer)
    cv2.putText(img, str(int(fps)), (75,50), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0,0,255), 2)
    
    #********** Image Stacking Horizontally
    #hStack = np.hstack([result, mask])
    imgStack = fnc.stackImages(0.6,([result,mask],[img,imgHsv]))
    
    cv2.imshow("Stacked Images", imgStack)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()










