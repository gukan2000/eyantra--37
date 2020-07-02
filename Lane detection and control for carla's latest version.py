import glob
import os
import sys
import random
import time
import numpy as np
import cv2
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import math
IM_WIDTH = 640
IM_HEIGHT = 480
SECONDS_PEREPISODE = 10
class carenv:
    SHOW_PREVIEW  = True
    STEER_AMT  = 1.0
    im_width = IM_WIDTH
    im_height = IM_HEIGHT
    front_camera = None
    throttle = 0.0
    steer = 0.0
    count = 0
    def __init__(self):
        self.client = carla.Client("localhost",2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.bp = self.blueprint_library.filter('prius')[0]
    def make_lines(self,image,average):# this returns 2 points if we feed slope,intercept
        slope , intercept = average
        y1 = 429
        y2 = int(y1*0.6)
        x1 = int((y1- intercept)/slope)
        x2 = int((y2- intercept)/slope)
        return x1,y1,x2,y2
    def make_lines1(self,image,average):
        slope , intercept = average
        y1 = 0
        y2 = int(image.shape[0])
        x1 = int((y1- intercept)/slope)
        x2 = int((y2- intercept)/slope)
        return x1,y1,x2,y2   
    def averaged_lines(self,image,lines):# this line creates the average lines on the left side and right side from those two average lines we taking 2 points each from line with same y cordinates and we taking average of their x cordinates two get two points required for a line
        left_fit = []
        right_fit = []
        img = np.zeros_like(image)
        if lines is not None:
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
            x1,y1,x2,y2 = self.make_lines(image,left_average)
            X1 = X1 + x1
            X2 = X2 + x2
            Y1 = Y1 + y1
            Y2 = Y2 + y2
            #cv2.line(img,(x1,y1),(x2,y2),(255,0,0),5)
            x1,y1,x2,y2 = self.make_lines(image,right_average)
            X1 = X1 + x1
            X2 = X2 + x2
            Y1 = Y1 + y1
            Y2 = Y2 + y2
            #cv2.line(img,(x1,y1),(x2,y2),(255,0,0),5)
            X1 = int(X1/2)
            X2 = int (X2/2)
            Y1 = int(Y1/2)
            Y2 = int(Y2/2)
            #cv2.line(img,(X1,Y1),(X2,Y2),(255,0,0),5)
            if X2 - X1 != 0:
                m = (Y2 - Y1)/(X2-X1)
                b = Y1 - m*X1
                average = (m,b)
                a , b ,c,d = self.make_lines1(image, average)
            else:
                m = math.pi/2
                b = 0
                a,b,c,d = int(img.shape[1]/2),int(img.shape[0]), int(img.shape[1]/2),0
        else:
            m= math.pi/2
            b = 0 
            a,b,c,d = int(img.shape[1]/2),int(img.shape[0]), int(img.shape[1]/2),0
        if m<0:
            theta = -(90 + math.atan(m)*180/math.pi)
        else:
            if m>=0 :
                theta = (90 - math.atan(m)*180/math.pi)
        self.steer = 0.0
        if abs(theta)>=2.5:
            self.steer = 0.0
            self.steer = theta*1/90
        else:
            self.steer = 0
        cv2.line(img, (a,b), (c,d), (0,255,0), 5)
        cv2.line(img, (int(img.shape[1]/2),img.shape[0]), (int(img.shape[1]/2),0), (0,0,255),5)
        return cv2.addWeighted(img, 0.8, image, 1, 1)
    def preprocessing(self,image):#does preprocessing and returns edges in the images
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.Canny(image, 200, 250)
        return image
    def roi_right(self,image):#returns image which edges from the ROI alone
        height = image.shape[0]
        width = image.shape[1]
        roi = np.array([(273, 259),(45, height),(601, height),(361, 262)])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask,[roi],255)
        masked = cv2.bitwise_and(image, mask)
        return masked
    def display_lines(self,image,lines):# this function was used to check whether we are getting lines or not now this is not needed
        self.line_image = np.zeros_like(image)
        if self.lines is not None:
            for line in self.lines:
                x1,y1,x2,y2 = line.reshape(4)
                cv2.line(self.line_image,(x1,y1),(x2,y2),(255,0,0),5)
        return self.line_image 
    def process_image(self,image):
        self.i = np.array(image.raw_data)
        self.i2 = self.i.reshape((self.im_height,self.im_width,4))
        #i2 = i.reshape((800,600,4))
        self.i3 = self.i2[:,:,:3]
        self.i3p = self.preprocessing(self.i3)
        self.i3r = self.roi_right(self.i3p)
        self.lines = cv2.HoughLinesP(self.i3r,2,np.pi/180,20,np.array([]),minLineLength = 20,maxLineGap = 5)
        #self.line_image = self.display_lines(self.i3, self.lines)
        self.line_image =  self.averaged_lines(self.i3,self.lines)
        self.count = self.count + 1
        #print(self.vehicle.get_location())
        if self.SHOW_PREVIEW and self.count >= 20:
            cv2.imshow("",self.line_image)
            #cv2.imshow("1", self.i3)
            #cv2.imshow("2", self.i3p)
            #cv2.imshow("", self/i3r)
            cv2.waitKey(1)
        self.front_camera = self.i3
    def reset(self):
        self.collision_history = []
        self.actor_list = []
        self.spawn_point =  self.world.get_map().get_spawn_points()[43]    #self.spawn_point =  carla.Transform(carla.Location,y,0),carla.Rotation(0,degree,0))
        self.vehicle = self.world.spawn_actor(self.bp,self.spawn_point)
        self.actor_list.append(self.vehicle)
        #vehicle.set_autopilot(True)
        self.camera_bp = self.blueprint_library.find('sensor.camera.rgb')
        self.camera_bp.set_attribute("image_size_x",f"{self.im_width}")
        self.camera_bp.set_attribute("image_size_y",f"{self.im_height}")
        self.camera_bp.set_attribute("fov","110")
        self.relative_transform = carla.Transform(carla.Location(x=0,y = -4.5, z= 2))
        self.camera = self.world.spawn_actor(self.camera_bp, self.relative_transform, attach_to=self.vehicle)
        self.actor_list.append(self.camera)
        self.camera.listen(lambda image: self.process_image(image))
        self.collision_sensor_bp = self.blueprint_library.find('sensor.other.collision')
        self.sensor = self.world.spawn_actor(self.collision_sensor_bp,self.relative_transform,attach_to = self.vehicle)
        self.actor_list.append(self.sensor) 
        self.sensor.listen(lambda event: self.callback(event))
        while self.front_camera is None:
            time.sleep(0.01)
        self.episode_start = time.time()
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0,brake = 0.0))
    def callback(self, event):
        self.collision_history.append(event)
    def step(self,action):
        self.vehicle.apply_control(carla.VehicleControl(throttle = 0.3,steer =self.steer))
    def destroy(self):
        for self.actor in self.actor_list:
            self.actor.destroy()
            print("all cleaned up")
a = carenv()
a.reset()
current_time  = time.time()
while True:
    a.step(0)
    if time.time() - current_time > 10:
        print(time.time()-current_time)
        a.destroy()
        break

   