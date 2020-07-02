# eyantra--37
This repo contains my contribution in the project-37 of eyantra summer internship program
ImageTester:

The first script handles the job of lane detection on a image to get left and right lane and it also calculates the average of those lines to find the center of the lane. considering the vehicle is currently moving in the direction shown by the line which divides the image into two equal parts vertically we would be able to calculate the angle between the desirable and actual direction. After calculating that we have to convert it into steering scale.We would a output between -90 to 90 which we would map to -1 to 1 in ackerman steering scale.

Carla implementation for latest version:

It invovles converting the things done on a particular image to work on a contious camera feed. The latest version had a function camera.listen inside to which we can call a function to process a image as soon as the camera listens to recieve a image.Inside this processing function all the things which were done inside the first script is done again to process the image to return the steering angle.

Carla implementation for custom version:

It is mostly the script for gaining data made by Gaurav only but small tweaks are made on it make it automatic and the functions used in the first script is used here.

Lane_Tracker:

This script uses select ROI for choosing bounding boxes for lane. Then TrackerTLD is created and initialised with this bounding box and its corresponding image.Then we get centre of the bounding box everytime the image changes and the tracker gets updated.Then with center and the midpoint of the bottom line of the image(width/2,height) is used to create a line then the angle between this line and Y axis found.It is in the range of -90 to 90 then angle is mapped to -1 to 1 to give the steering value.

Lane_object_tracker:

The fifth script invovles addition of another Tracker for object detection so that the vehicle can move away from it. The above method used in Fourth script is used for getting steering values from the object detector(Note: it is important to note that the steering value we get from the object detector is not the value we should cause if the object is in say left then it would give only negative steering value which would make the bot to move in left direction but that is not needed).if the object detector cant detect anything or the area of the bounding box of object detector is less than the AREA THRESHOLD then steering value given by lane dtector will be used if the area is above threshold then if value by object detector is less than the POSITION THRESHOLD then negative of the value given by object dtector will be fed to the bot/ vehicle if it is above the POSITION THRESHOLD then value from lane detector will be used.

AREA THRESHOLD: The area threshold is dtermined by keeping the object at a save distance from the bot then its area is caluculated through width and height unpacked from initial bbox for object.If the area is above this threshold which means the object is neare to the bot than save distance so it is a danger and the bot should move away from it.

POSITION THREHOLD: this threshold determines whether the object is within the lane or not . If a object is not within the lane then we need not worry about it.

FinalIP:

Position threshold is given in such a way that object tracker's steering value must lie within left end and right end of lane tracker. It is done by the following way while finding steering value given by the center of lane detector we would be giving it to the bot in the cases where object tracker can't find the object or the object's area is below area threshold. If object tracker's area is above threshold then we will take steering values for the midpoints of the breath lines of the lane tracker and we name them steerl and steerr which corresponds to left and right midpoints of the breath lines in tracker. We will then check whether steering value given by center of object tracker lies between steerl and steerr if it lies between that then final steering angle of steerl + steerr - steero would be given this will ensure that always the turning will decrease as we will move away from the obstacle. If the object tracker's steering value doesn't lie between the steerl and steerr which means it is not on the lane so , lane detector's output will be given as final output. colour tagged everything such that the steering output will be green in colour if object is not visible or it is within area threshold , it will be of blue in colour if the obstacle is inside the lane and will be red in colour if obstacle is outside the lane.

multilane: Implemented for multilane.All the cases in the above script will apply but the only diffence is that we would need 3 trackers two for lane(left,right), 1 for object and if object is detected bot will switch lanes.
