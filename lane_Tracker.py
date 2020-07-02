import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
# bbox is bounding box for lane

def drawBox(img,bbox):# this function is used in case of lane detection
    x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    X, Y = int(x+w/2) , int(y+ h/2) # center of the lane detector
    cv2.rectangle(img, (x, y), ((x + w), (y + h)), (255, 0, 255), 3, 3 )
    cv2.circle(img, (X,Y), 2, (255,0,0),-1)
    cv2.line(img, (X,Y), (int(img.shape[1]/2),img.shape[0]), (0,255,0))
    if (X - (img.shape[1]/2)) != 0: 
        theta = math.atan((Y-img.shape[0])/(X - (img.shape[1]/2))) # angle with respect to x axis 
    else:
        theta = 0
    theta = theta*180/math.pi
    if theta>0:
        theta = -(90 - theta)
    else:
        theta = (90 + theta)
    # now theta would be with respect to y axis
    steer = theta/90# into steering value
    cv2.putText(img, str(steer), (320, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
tracker = cv2.TrackerTLD_create()# tracker for lane created
frame = cv2.imread("G:\\CarlaSimulator\\resize\\img0.jpg")
bbox = cv2.selectROI("Tracking",frame, False)
print(bbox)
#bbox = (258, 430, 301, 74)
tracker.init(frame, bbox)
print(tracker.init(frame, bbox))
count = 1
while count <=479:
    img = cv2.imread("G:\\CarlaSimulator\\resize\\img" + str(count)+ ".jpg")
    # canny=canny_image(frame)

    # ceopped_image=roi(canny)

    # line=cv2.HoughLinesP(ceopped_image,2,np.pi/180,50,np.array([]),minLineLength=100,maxLineGap=5) #the second and third argument of this function helps to contribute the size of 2D grid that is used
    #                                                                                                #second argument is rho(or number of rows) and the third argument is thetha(or number of coloumns)
    #                                                                                                #here, we have 2,1 matrix, 2rows and 1 coloumn(1 radian)
    # averaged_lines=average_slope_intercept(frame,line)
    # line_image=display_lines(frame,averaged_lines)
    # final=cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    timer = cv2.getTickCount()
    success, bbox = tracker.update(img) # update success True if lane is detected.
    if success:
        drawBox(img,bbox)
        cv2.putText(img, "Lost", (100, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
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


 
 
