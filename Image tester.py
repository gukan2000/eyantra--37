import cv2
import numpy as np
import math
def preprocessing(image):#does preprocessing and returns edges in the images
	image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	image = cv2.Canny(image, 100, 150)
	return image
def roi_right(image):#returns image which edges from the ROI alone
    height = image.shape[0]
    width = image.shape[1]
    #roi = np.array([(305, 327),(0, 403),(0,height),(width, height),(797, 411),(493, 318)])
    roi = np.array([(151, height),(673, height),(673, 431),(151, 445)])

    #roi = np.array([(292, 253),(76, height),(593, height),(357, 262)])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask,[roi],255)
    masked = cv2.bitwise_and(image, mask)
    return masked
def display_lines(image,lines):# this function was used to check whether we are getting lines or not now this is not needed
    line_image = np.zeros_like(image)
    for line in lines:
        x1,y1,x2,y2 = line.reshape(4)
        cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),5)
    return line_image
def mouse_drawing(event, x, y, flags, params):# this was used to mark roi points not needed now
    if event == cv2.EVENT_LBUTTONDOWN:
        print((x,y))
def averaged_lines(image,lines):# this line creates the average lines on the left side and right side from those two average lines we taking 2 points each from line with same y cordinates and we taking average of their x cordinates two get two points required for a line
    left_fit = []
    right_fit = []
    for line in lines:
        x1,y1,x2,y2 = line.reshape(4)
        slope , intercept = np.polyfit((x1,x2),(y1,y2),1)

        if slope<0:
            left_fit.append((slope,intercept))
        else:
            right_fit.append((slope,intercept))
    left_average = np.average(left_fit,axis =0)
    right_average = np.average(right_fit,axis =0)
    X1 = 0
    Y1 = 0
    X2 = 0
    Y2 = 0
    img = np.zeros_like(image)
    x1,y1,x2,y2 = make_lines(image,left_average)
    X1 = X1 + x1
    X2 = X2 + x2
    Y1 = Y1 + y1
    Y2 = Y2 + y2
    cv2.line(img,(x1,y1),(x2,y2),(255,0,0),5)
    x1,y1,x2,y2 = make_lines(image,right_average)
    X1 = X1 + x1
    X2 = X2 + x2
    Y1 = Y1 + y1
    Y2 = Y2 + y2
    cv2.line(img,(x1,y1),(x2,y2),(255,0,0),5)
    X1 = int(X1/2)
    X2 = int (X2/2)
    Y1 = int(Y1/2)
    Y2 = int(Y2/2)
    #cv2.line(img,(X1,Y1),(X2,Y2),(255,0,0),5)
    if X2-X1 !=0:
    	m = (Y2 - Y1)/(X2-X1)
    else:
    	m = (Y2-Y1)/0.00001
    b = Y1 - m*X1
    average = (m,b)
    a , b ,c,d = make_lines1(image, average)
    if m<0:
    	theta = -( 90 + math.atan(m)*180/math.pi)
    elif m>=0 :
    	theta =  90 - math.atan(m)*180/math.pi
    print(theta)
    cv2.line(img, (a,b), (c,d), (0,255,0), 5)
    cv2.line(img, (int(img.shape[1]/2),img.shape[0]), (int(img.shape[1]/2),0), (0,0,255),5)
    return img
def make_lines(image,average):# this returns 2 points if we feed slope,intercept
    slope , intercept = average
    y1 = 429
    y2 = int(y1*0.6)
    x1 = int((y1- intercept)/slope)
    x2 = int((y2- intercept)/slope)
    return x1,y1,x2,y2
def make_lines1(image,average):
    slope , intercept = average
    y1 = 0
    y2 = int(image.shape[0])
    x1 = int((y1- intercept)/slope)
    x2 = int((y2- intercept)/slope)
    return x1,y1,x2,y2    
cv2.namedWindow("")
cv2.setMouseCallback("",mouse_drawing)
image = cv2.imread("G:\\CarlaSimulator\\images\\"+str(6284)+".png")
#print(image)
processed = roi_right(preprocessing(image))
cv2.imshow('2',preprocessing(image))
cv2.imshow('', image)
cv2.imshow('1',processed )
lines = cv2.HoughLinesP(processed,2,np.pi/180,20,np.array([]),minLineLength = 10,maxLineGap = 5)
line_image = display_lines(image, lines)
cv2.imshow('2', line_image)
line_image1 = averaged_lines(image, lines)
result = cv2.addWeighted(image, 1, line_image1, 0.8, 1)
cv2.imshow('3', result)
cv2.waitKey(0)
cv2.destroyAllWindows()