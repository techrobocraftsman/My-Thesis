import cv2
import numpy as np
import functions as fnc

frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)

def empty(a):
    pass

# Creating Trackbars
cv2.namedWindow("HSV")
cv2.resizeWindow("HSV", 640, 100)
#cv2.resizeWindow("HSV", 200, 100)

cv2.createTrackbar("HUE Min", "HSV", 0, 190, empty)
cv2.createTrackbar("HUE Max", "HSV", 190, 190, empty)
cv2.createTrackbar("SAT Min", "HSV", 0, 255, empty)
cv2.createTrackbar("SAT Max", "HSV", 255, 255, empty)
cv2.createTrackbar("VALUE Min", "HSV", 0, 255, empty)
cv2.createTrackbar("VALUE Max", "HSV", 255, 255, empty)


while True:
    timer = cv2.getTickCount()
    success, img = cap.read()
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    #***********Get trackbar Values    
    h_min = cv2.getTrackbarPos("HUE Min", "HSV")
    h_max = cv2.getTrackbarPos("HUE Max", "HSV")
    s_min = cv2.getTrackbarPos("SAT Min", "HSV")
    s_max = cv2.getTrackbarPos("SAT Max", "HSV")
    v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
    v_max = cv2.getTrackbarPos("VALUE Max", "HSV")
    #print(h_min)
    
    # ********** Masking The Image
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(imgHsv, lower, upper)
    result = cv2.bitwise_and(img, img, mask=mask)
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    #********** FPS calculations
    fps = cv2.getTickFrequency()/(cv2.getTickCount()-timer)
    cv2.putText(img, str(int(fps)), (75,50), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0,0,255), 2)
    
    #********** Image Stacking Horizontally
    #hStack = np.hstack([result, mask])
    #imgStack = fnc.stackImages(0.6,([result,mask],[img,imgHsv]))
    imgStack = fnc.stackImages(0.6,([result,mask]))
    
    cv2.imshow("Stacked Images", imgStack)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()









