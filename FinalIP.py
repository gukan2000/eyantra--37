import cv2
import numpy as np
import matplotlib.pyplot as plt
import math


def steer1(x,y,img): # had to do this operation again and again for many points so created a seperate function for it(this will give steering angle for that particular point)

    if (x - (img.shape[1]/2)) != 0: 
        theta = math.atan((y-img.shape[0])/(x - (img.shape[1]/2)))
    else:
        theta = 0
    theta = theta*180/math.pi
    if theta>0:
        theta = -(90 - theta)
    else:
        theta = (90 + theta)
    return theta/90
def drawBox(img,bbox):# this function will be used if no object is detected only lane is detected
    x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    X, Y = int(x+w/2) , int(y+ h/2) # here we find the center of the lane tracker
    cv2.rectangle(img, (x, y), ((x + w), (y + h)), (255, 0, 255), 3, 3 )
    cv2.circle(img, (X,Y), 2, (255,0,0),-1)
    cv2.line(img, (X,Y), (int(img.shape[1]/2),img.shape[0]), (0,255,0))# these functions are just for visualisation
    steer = steer1(X, Y, img)
    cv2.putText(img, str(steer), (320, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

def drawBox1(img,bbox,bbox1):# this will be used if both lane and object are detected
    x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    X, Y = int(x+w/2) , int(y+ h/2)
    x1, y1, w1, h1 = int(bbox1[0]), int(bbox1[1]), int(bbox1[2]), int(bbox1[3])
    X1, Y1 = int(x1+w1/2) , int(y1+ h1/2)
    steer2 = steer1(X, Y, img)# here both center of object and lane trackers or calculated 
    a = w1*h1
    if a<600:# checks whether object tracker is above the area threshold or not.if not  then value by lane tracker would be used 
        cv2.putText(img, str(steer2), (320, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:# if it is above the threshold then steering values corresponding to the midpoints of the breath lines of the lane tracker rectangle will be calculated, the value corresponding to the centre of the object will also be calculated
        steero = steer1(X1, Y1, img)#steering value for center of object tracker.
        steerl = steer1(x, y/2, img)# steering value of midpoint of left breath line
        steerr = steer1(x+w, y/2, img)# for right line
        if (steero>steerl and steero<steerr):# checks whether steero lies between steerl and steer r or not which implies that whether object lies within lane or not 
            steer = steerr + steerl - steero# if yes the following steering value is used.Why this will ensure that the vehicle will move away and also that as it is moving the final steering value will decrease.
            cv2.putText(img, str(steer), (320, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        else:
            cv2.putText(img, str(steer2), (320, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0 , 255), 2)# this means object is not in the lane which implies lane dtector's value will be given

        
    cv2.rectangle(img, (x, y), ((x + w), (y + h)), (255, 0, 255), 3, 3 )
    cv2.circle(img, (X,Y), 2, (255,0,0),-1)
    cv2.line(img, (X,Y), (int(img.shape[1]/2),img.shape[0]), (0,255,0))
    cv2.rectangle(img, (x1, y1), ((x1 + w1), (y1 + h1)), (255, 0, 255), 3, 3 )
    cv2.circle(img, (X1,Y1), 2, (255,0,0),-1)
    cv2.line(img, (X1,Y1), (int(img.shape[1]/2),img.shape[0]), (0,0,255))


    

image=cv2.imread('test.jpg')
lane=np.copy(image)

def canny_image(image):# from line 58 to 109 is not used in this script.                                   #function to get canny output
    grey=cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
    blur=cv2.GaussianBlur(grey,(5,5),0)
    canny=cv2.Canny(grey,0,30)
    return canny

def roi(image):                                                 #Defination of region of interest
    height=image.shape[0]
    triangles=np.array([[(0,height),(1100,height),(520,0)]])
    mask=np.zeros_like(image)
    cv2.fillPoly(mask,triangles,255)
    masked_image=cv2.bitwise_and(image,mask)
    return masked_image

def display_lines(images,lines):                               #function to display detected line
    line_image=np.zeros_like(images)
    if lines is not None:

        for line in lines:
            x1,y1,x2,y2=line.reshape(4)
            cv2.line(line_image,(x1,y1),(x2,y2),(0,255,0),10)
    return line_image

def make_cordinate(image,line_parameters):                   #get coordinates of line
    slope,intercept=line_parameters
    y1=image.shape[0]
    y2=int(y1*(3/5))
    x1=int((y1-intercept)/slope)
    x2=int((y2-intercept)/slope)
    return np.array([x1,y1,x2,y2])


def average_slope_intercept(image,lines):                      #Make a average line from many lines
    left_fit=[]
    right_fit=[]
    for line in lines:
        x1,y1,x2,y2=line.reshape(4)
        parameters=np.polyfit((x1,x2),(y1,y2),1)
        slope=parameters[0]
        intercept=parameters[1]
        if slope<0:
            left_fit.append((slope,intercept))
        else:
            right_fit.append((slope,intercept))
    average_left=np.average(left_fit,axis=0)
    average_right=np.average(right_fit,axis=0)
    left_line=make_cordinate(image,average_left)
    right_line=make_cordinate(image,average_right)
    return np.array([left_line,right_line])# from line 58 to 109 is not used in this script.




tracker = cv2.TrackerTLD_create()#tracker for lane is created
print(type(tracker))
tracker1 = cv2.TrackerTLD_create()# tracker for object is created
frame = cv2.imread("G:\\CarlaSimulator\\resize3\\img112.jpg")# image for selecting initial bounding-box for lane detector
frame1 = cv2.imread("G:\\CarlaSimulator\\resize3\\img283.jpg")# image for selecting initial bounding-box for object detector
bbox = cv2.selectROI("Tracking",frame, False)
print(bbox)
bbox1 = cv2.selectROI("Tracking",frame1, False)# bbox for lane and bbox1 for object
print(bbox1)

#bbox = (258, 430, 301, 74)
tracker.init(frame, bbox)
tracker1.init(frame1, bbox1)# initialisation
print(tracker.init(frame, bbox))
count = 1
# images taken from the video and got resized and got stored in resized folder is then traversed one by one
while count <=539:
    img = cv2.imread("G:\\CarlaSimulator\\resize3\\img" + str(count)+ ".jpg")
    # canny=canny_image(frame)

    # ceopped_image=roi(canny)

    # line=cv2.HoughLinesP(ceopped_image,2,np.pi/180,50,np.array([]),minLineLength=100,maxLineGap=5) #the second and third argument of this function helps to contribute the size of 2D grid that is used
    #                                                                                                #second argument is rho(or number of rows) and the third argument is thetha(or number of coloumns)
    #                                                                                                #here, we have 2,1 matrix, 2rows and 1 coloumn(1 radian)
    # averaged_lines=average_slope_intercept(frame,line)
    # line_image=display_lines(frame,averaged_lines)
    # final=cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    timer = cv2.getTickCount()
    success, bbox = tracker.update(img) 
    success1,bbox1 = tracker1.update(img)# for each update the bbox has to be changed this update will give info about whether we r able track the object in the particular frame or not with a bool value
    if success:# if get lane tracker to track the lane
        if not success1:# if the object is not visible 
            drawBox(img,bbox)
            cv2.putText(img, "Lost", (100, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
 
        else:# if the object is visible 
            drawBox1(img,bbox, bbox1)
    cv2.rectangle(img,(15,15),(200,90),(255,0,255),2)
    cv2.putText(img, "Fps:", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,255), 2);
    cv2.putText(img, "Status:", (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2);
 
 
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
    if fps>60: myColor = (20,230,20)
    elif fps>20: myColor = (230,20,20)
    else: myColor = (20,20,230)
    cv2.putText(img,str(int(fps)), (75, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, myColor, 2);
    cv2.imshow("result",img)
    count = count + 1
    if cv2.waitKey(1)==ord('q'):
        break
cap.release()
cv2.destroyAllWindows()