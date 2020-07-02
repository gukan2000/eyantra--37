import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
# bbox for left lane 
# bbox1 for object 
# bbox2 for right lane
# switch is used to identify which lane on which lane it is travelling right now as the bot will be placed in left side so the value will be initialised to -1 and right lane has a value 1
# A is the area of the bounding box of the object tracker after drawing it on a image where the object is kept in a safe distance that will act as area threshold.

def steer1(x,y,img): # had to do this operation again and again for many points so created a seperate function for it(this will give steering angle for that particular point)

    if (img.shape[0]-y) != 0: 
        theta = math.atan((x - (img.shape[1]/2))/(img.shape[0]-y))
    else:
        if (x-(img.shape[1]/2)) > 0:
            theta = math.pi/2
        else:
            theta = -math.pi/2
    theta = theta*2/math.pi
    return theta
def drawBox(img,bbox):# this function will be used if no object is detected only lane is detected
    x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    X, Y = int(x+w/2) , int(y+ h/2) # here we find the center of the lane tracker
    cv2.rectangle(img, (x, y), ((x + w), (y + h)), (255, 0, 255), 3, 3 )
    cv2.circle(img, (X,Y), 2, (255,0,0),-1)
    cv2.line(img, (X,Y), (int(img.shape[1]/2),img.shape[0]), (0,255,0))# these functions are just for visualisation
    steer = steer1(X, Y, img)
    cv2.putText(img, str(steer), (320, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

def drawBox1(img,bbox,bbox1,switch,bbox2,A):# this will be used if both lane and object are detected
    if switch == -1# unpacking left lane tracker if switch = -1
        x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    else:# unpacking left lane tracker if switch = 1
        x, y, w, h = int(bbox2[0]), int(bbox2[1]), int(bbox2[2]), int(bbox2[3])
    X, Y = int(x+w/2) , int(y+ h/2)
    x1, y1, w1, h1 = int(bbox1[0]), int(bbox1[1]), int(bbox1[2]), int(bbox1[3])
    X1, Y1 = int(x1+w1/2) , int(y1+ h1/2)
    steer2 = steer1(X, Y, img)# here both center of object and lane trackers or calculated 
    a = w1*h1 # 
    if a<A:# checks whether object tracker is above the area threshold or not.if not  then value by lane tracker would be used 
        cv2.putText(img, str(steer2), (320, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:# if it is above the threshold then steering values corresponding to the midpoints of the breath lines of the lane tracker rectangle will be calculated, the value corresponding to the centre of the object will also be calculated
        steero = steer1(X1, Y1, img)#steering value for center of object tracker.
        steerl = steer1(x, y/2, img)# steering value of midpoint of left breath line
        steerr = steer1(x+w, y/2, img)# steering value of midpoint of right breath line
        if steero < steerr && steero >= steerl:
            switch = -switch # switch will be inverted if object detected 
            switch_threshold = switch*(steerr-steerl)/2#this is done to trun the bot so it can see the switched lane
            cv2.putText(img, str(steer2 + switch_threshold), (320, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(img, str(steer2), (320, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)# this means object is not in the lane which implies lane dtector's value will be given    
    cv2.rectangle(img, (x, y), ((x + w), (y + h)), (255, 0, 255), 3, 3 )
    cv2.circle(img, (X,Y), 2, (255,0,0),-1)
    cv2.line(img, (X,Y), (int(img.shape[1]/2),img.shape[0]), (0,255,0))
    cv2.rectangle(img, (x1, y1), ((x1 + w1), (y1 + h1)), (255, 0, 255), 3, 3 )
    cv2.circle(img, (X1,Y1), 2, (255,0,0),-1)
    cv2.line(img, (X1,Y1), (int(img.shape[1]/2),img.shape[0]), (0,0,255))
    return switch

tracker = cv2.TrackerTLD_create()#tracker for left_lane is created
tracker1 = cv2.TrackerTLD_create()# tracker for object is created
tracker2 = cv2.TrackerTLD_create()#tracker for right_lane is created
frame = cv2.imread("G:\\CarlaSimulator\\resize4\\img0.jpg")# image for selecting initial bounding-box for lane detector
frame1 = cv2.imread("G:\\CarlaSimulator\\resize1\\img1005.jpg")# image for selecting initial bounding-box for object detector
frame2 = cv2.imread("G:\\CarlaSimulator\\resize4\\img1001.jpg")
bbox = cv2.selectROI("Tracking",frame, False)
print(bbox)
bbox1 = cv2.selectROI("Tracking",frame1, False)# bbox for lane and bbox1 for object
#print(bbox1)
A = int(bbox1[2])*int(bbox1[3])
bbox2 = cv2.selectROI("Tracking",frame2, False)
#bbox = (258, 430, 301, 74)
tracker.init(frame, bbox)
tracker1.init(frame1, bbox1)
tracker2.init(frame2,bbox2)# initialisation
count = 1
# images taken from the video and got resized and got stored in resized folder is then traversed one by one
switch = -1 
while count <=539:
    img = cv2.imread("G:\\CarlaSimulator\\resize4\\img" + str(count)+ ".jpg")
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
    success1,bbox1 = tracker1.update(img)
    success2, bbox2 = tracker2.update(img) #for each update the bbox has to be changed this update will give info about whether we r able track the object in the particular frame or not with a bool value
    if success or success2:# if get lane tracker to track the lane
        if not success1:# if the object is not visible 
            if switch == -1:
                drawBox(img,bbox)
            else:
                drawBox(img, bbox2)
            cv2.putText(img, "Lost", (100, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
 
        else:# if the object is visible
            switch = drawBox1(img,bbox, bbox1, switch,bbox2,A)
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
