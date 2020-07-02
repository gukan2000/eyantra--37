import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
# bbox is bounding box of lane tracker
# bbox1 is bounding box of object tracker
# A is area of the initial bbox of object which would taken from a image where object is in safe distance

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

def drawBox1(img,bbox,bbox1,A):# this will be used if both lane and object are detected
    x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    X, Y = int(x+w/2) , int(y+ h/2)
    x1, y1, w1, h1 = int(bbox1[0]), int(bbox1[1]), int(bbox1[2]), int(bbox1[3])
    X1, Y1 = int(x1+w1/2) , int(y1+ h1/2)
    steer2 = steer1(X, Y, img)# here both center of object and lane trackers or calculated 
    a = w1*h1
    if a<A:# checks whether object tracker is above the area threshold or not.if not  then value by lane tracker would be used 
        cv2.putText(img, str(steer2), (320, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:# if it is above the threshold then steering values corresponding to the midpoints of the breath lines of the lane tracker rectangle will be calculated, the value corresponding to the centre of the object will also be calculated
        steero = steer1(X1, Y1, img)#steering value for center of object tracker.
        steerl = steer1(x, y/2, img)# steering value of midpoint of left breath line
        steerr = steer1(x+w, y/2, img)# for right line
        if (steero>steerl and steero<steerr):# checks whether steero lies between steerl and steer r or not which implies that whether object lies within lane or not 
            steer = steerr + steerl - steero# if yes the following steering value is used.Why this will ensure that the vehicle will move away and also that as it is moving the final steering value will decrease.
            cv2.putText(img, str(steer), (320, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            cv2.putText(img, str(steer2), (320, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0 , 0), 2)# this means object is not in the lane which implies lane dtector's value will be given

        
    cv2.rectangle(img, (x, y), ((x + w), (y + h)), (255, 0, 255), 3, 3 )
    cv2.circle(img, (X,Y), 2, (255,0,0),-1)
    cv2.line(img, (X,Y), (int(img.shape[1]/2),img.shape[0]), (0,255,0))
    cv2.rectangle(img, (x1, y1), ((x1 + w1), (y1 + h1)), (255, 0, 255), 3, 3 )
    cv2.circle(img, (X1,Y1), 2, (255,0,0),-1)
    cv2.line(img, (X1,Y1), (int(img.shape[1]/2),img.shape[0]), (0,0,255))
    
tracker = cv2.TrackerTLD_create()#tracker for lane is created
tracker1 = cv2.TrackerTLD_create()# tracker for object is created
frame = cv2.imread("G:\\CarlaSimulator\\resize1\\img" + str(0) + ".jpg") 
frame1 = cv2.imread("G:\\CarlaSimulator\\resize1\\img" + str(384) + ".jpg") 
bbox = cv2.selectROI("Tracking",frame, False)
print(bbox)
bbox1 = cv2.selectROI("Tracking",frame1, False)# bbox for lane and bbox1 for object
print(bbox1)
A = int(bbox1[2])*int(bbox1[3])
#bbox = (258, 430, 301, 74)
tracker.init(frame, bbox)
tracker1.init(frame1, bbox1)# initialisation
print(tracker.init(frame, bbox))
count = 1
# images taken from the video and got resized and got stored in resized folder is then traversed one by one
while count <=539:
    img = cv2.imread("G:\\CarlaSimulator\\resize1\\img" + str(count)+ ".jpg")
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
            drawBox(img_rgb,bbox)
            cv2.putText(img, "Lost", (100, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
 
        else:# if the object is visible 
            drawBox1(img,bbox, bbox1,A)
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
