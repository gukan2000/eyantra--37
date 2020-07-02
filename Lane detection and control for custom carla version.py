#!/usr/bin/env python3

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Keyboard controlling for CARLA. Please refer to client_example.py for a simpler
# and more documented example.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot

    R            : restart level

STARTING in a moment...
"""

from __future__ import print_function

import argparse
import logging
import random
import time
import cv2
import pandas as pd
import math
i=0
name_dict = {
            'Image': [],
            'Steer': [],
            'throttle':[]
          }
st=[]
th=[]
imgg=[]
try:
    import pygame
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

from carla import image_converter
from carla import sensor
from carla.client import make_carla_client, VehicleControl
from carla.planner.map import CarlaMap
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.util import print_over_same_line
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
MINI_WINDOW_WIDTH = 320
MINI_WINDOW_HEIGHT = 180


def make_carla_settings(args):
    """Make a CarlaSettings object with the settings we need."""
    settings = CarlaSettings()
    settings.set(
        SynchronousMode=True,
        SendNonPlayerAgentsInfo=True,
        NumberOfVehicles=15,
        NumberOfPedestrians=30,
        WeatherId=random.choice([1, 3, 7, 8, 14]),
        QualityLevel=args.quality_level)
    settings.randomize_seeds()
    camera0 = sensor.Camera('CameraRGB')
    camera0.set_image_size(WINDOW_WIDTH, WINDOW_HEIGHT)
    camera0.set_position(2.0, 0.0, 10.3)
    camera0.set_rotation(0.0, 0.0, 0.0)
    settings.add_sensor(camera0)
    camera1 = sensor.Camera('CameraDepth', PostProcessing='Depth')
    camera1.set_image_size(MINI_WINDOW_WIDTH, MINI_WINDOW_HEIGHT)
    camera1.set_position(2.0, 0.0, 1.4)
    camera1.set_rotation(0.0, 0.0, 0.0)
    settings.add_sensor(camera1)
    camera2 = sensor.Camera('CameraSemSeg', PostProcessing='SemanticSegmentation')
    camera2.set_image_size(MINI_WINDOW_WIDTH, MINI_WINDOW_HEIGHT)
    camera2.set_position(2.0, 0.0, 1.4)
    camera2.set_rotation(0.0, 0.0, 0.0)
    settings.add_sensor(camera2)
    if args.lidar:
        lidar = sensor.Lidar('Lidar32')
        lidar.set_position(0, 0, 2.5)
        lidar.set_rotation(0, 0, 0)
        lidar.set(
            Channels=32,
            Range=50,
            PointsPerSecond=100000,
            RotationFrequency=10,
            UpperFovLimit=10,
            LowerFovLimit=-30)
        settings.add_sensor(lidar)
    return settings


class Timer(object):
    def __init__(self):
        self.step = 0
        self._lap_step = 0
        self._lap_time = time.time()

    def tick(self):
        self.step += 1

    def lap(self):
        self._lap_step = self.step
        self._lap_time = time.time()

    def ticks_per_second(self):
        return float(self.step - self._lap_step) / self.elapsed_seconds_since_lap()

    def elapsed_seconds_since_lap(self):
        return time.time() - self._lap_time


class CarlaGame(object):
    steer = 0.0   
    def __init__(self, carla_client, args):
        self.client = carla_client
        self._carla_settings = make_carla_settings(args)
        self._timer = None
        self._display = None
        self._main_image = None
        self._mini_view_image1 = None
        self._mini_view_image2 = None
        self._enable_autopilot = args.autopilot
        self._lidar_measurement = None
        self._map_view = None
        self._is_on_reverse = False
        self._display_map = args.map
        self._city_name = None
        self._map = None
        self._map_shape = None
        self._map_view = None
        self._position = None
        self._agent_positions = None
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
        y2 = int(image.shape[0]* 0.5)
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
            if X2-X1 !=0:
                m = (Y2 - Y1)/(X2-X1)
            else:
                m = (Y2-Y1)/0.00001
            b = Y1 - m*X1
            average = (m,b)
            a , b ,c,d = self.make_lines1(image, average)
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
        if abs(theta)>=0:
            self.steer = 0.0
            self.steer = theta*1/90
        #else:
        #   self.steer = 0
        cv2.line(img, (a,b), (c,d), (0,255,0), 5)
        cv2.line(img, (int(img.shape[1]/2),img.shape[0]), (int(img.shape[1]/2),0), (0,0,255),5)
        return cv2.addWeighted(img, 0.8, image, 1, 1)
    def preprocessing(self,image):#does preprocessing and returns edges in the images
        cv2.imshow("imagep",image)
        rgb_planes = cv2.split(image)
        result_planes = []
        result_norm_planes = []
        for plane in rgb_planes:
            dilated_img = cv2.dilate(plane, np.ones((7, 7), np.uint8))
            bg_img = cv2.medianBlur(dilated_img, 21)
            diff_img = 255 - cv2.absdiff(plane, bg_img)
            norm_img = cv2.normalize(diff_img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
            result_planes.append(diff_img)
            result_norm_planes.append(norm_img)
        result = cv2.merge(result_planes)
        image= cv2.merge(result_norm_planes)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.Canny(image, 200, 250)
        cv2.imshow("image",image)
        return image
    def roi_right(self,image):#returns image which edges from the ROI alone
        height = image.shape[0]
        width = image.shape[1]
        roi = np.array([(151, height),(673, height),(673, 431),(151, 445)])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask,[roi],255)
        masked = cv2.bitwise_and(image, mask)
        return masked
    def roi_center(self,image):
        height = image.shape[0]
        width = image.shape[1]
        roi = np.array([])
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
        #i2 = i.reshape((800,600,4))
        self.img1 = self.preprocessing(image)
        self.i3r = self.roi_right(self.img1)
        self.lines = cv2.HoughLinesP(self.i3r,2,np.pi/180,20,np.array([]),minLineLength = 10,maxLineGap = 10)
        self.line_image =  self.averaged_lines(image,self.lines) 
        return self.line_image

    def execute(self):
        """Launch the PyGame."""
        pygame.init()
        self._initialize_game()
        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return
                self._on_loop()
                self._on_render()
        finally:
            pygame.quit()

    def _initialize_game(self):
        self._on_new_episode()

        if self._city_name is not None:
            self._map = CarlaMap(self._city_name, 0.1643, 50.0)
            self._map_shape = self._map.map_image.shape
            self._map_view = self._map.get_map(WINDOW_HEIGHT)

            extra_width = int((WINDOW_HEIGHT/float(self._map_shape[0]))*self._map_shape[1])
            self._display = pygame.display.set_mode(
                (WINDOW_WIDTH + extra_width, WINDOW_HEIGHT),
                pygame.HWSURFACE | pygame.DOUBLEBUF)
        else:
            self._display = pygame.display.set_mode(
                (WINDOW_WIDTH, WINDOW_HEIGHT),
                pygame.HWSURFACE | pygame.DOUBLEBUF)

        logging.debug('pygame started')

    def _on_new_episode(self):
        self._carla_settings.randomize_seeds()
        self._carla_settings.randomize_weather()
        scene = self.client.load_settings(self._carla_settings)
        if self._display_map:
            self._city_name = scene.map_name
        number_of_player_starts = len(scene.player_start_spots)
        player_start = np.random.randint(number_of_player_starts)
        print('Starting new episode...')
        self.client.start_episode(player_start)
        self._timer = Timer()
        self._is_on_reverse = False

    def _on_loop(self):
        self._timer.tick()
        global i
        measurements, sensor_data = self.client.read_data()
        self._main_image = sensor_data.get('CameraRGB', None)
        control = self._get_keyboard_control(pygame.key.get_pressed())
        throttle=control.throttle
        steer=control.steer
        st.append(steer)
        th.append(throttle)
        self.img = image_converter.to_rgb_array(self._main_image)
        self.img1 = self.process_image(self.img)
        loc="Data/%s.jpg" % (i)
        imgg.append(loc)
        loc = "G:\\CarlaSimulator\\images\\"+str(i)+".png"
        cv2.imwrite(loc,self.img)
        i = i + 1
        # self._mini_view_image1 = sensor_data.get('CameraDepth', None)
        # self._mini_view_image2 = sensor_data.get('CameraSemSeg', None)
        # self._lidar_measurement = sensor_data.get('Lidar32', None)

        # Print measurements every second.
        if self._timer.elapsed_seconds_since_lap() > 1.0:
            if self._city_name is not None:
                # Function to get car position on map.
                map_position = self._map.convert_to_pixel([
                    measurements.player_measurements.transform.location.x,
                    measurements.player_measurements.transform.location.y,
                    measurements.player_measurements.transform.location.z])
                # Function to get orientation of the road car is in.
                lane_orientation = self._map.get_lane_orientation([
                    measurements.player_measurements.transform.location.x,
                    measurements.player_measurements.transform.location.y,
                    measurements.player_measurements.transform.location.z])

                self._print_player_measurements_map(
                    measurements.player_measurements,
                    map_position,
                    lane_orientation)
            else:
                self._print_player_measurements(measurements.player_measurements)

            # Plot position on the map as well.

            self._timer.lap()
        # Set the player position
        if self._city_name is not None:
            self._position = self._map.convert_to_pixel([
                measurements.player_measurements.transform.location.x,
                measurements.player_measurements.transform.location.y,
                measurements.player_measurements.transform.location.z])
            self._agent_positions = measurements.non_player_agents
        if control is None:
            self._on_new_episode()
        elif self._enable_autopilot:
            self.client.send_control(measurements.player_measurements.autopilot_control)
        else:
            self.client.send_control(control)

    def _get_keyboard_control(self, keys):
        """
        Return a VehicleControl message based on the pressed keys. Return None
        if a new episode was requested.
        """
        if keys[K_r]:
            return None
        control = VehicleControl()
        axis0 = joystick.get_axis(0)
        axis1 = joystick.get_axis(1)
        axis2= joystick.get_axis(2)
        axis3= joystick.get_axis(3)
        axis4 = joystick.get_axis(4)
        button = joystick.get_button(1)
        button1 = joystick.get_button(4)

        if abs(axis0) <0.15:
            axis0=0
        axis0=axis0/3
        if button1 is 1:
            control.steer = self.steer
        else:
            control.steer = axis0
        if button==1:
            print(button)
            self._is_on_reverse = not self._is_on_reverse
            time.sleep(0.0001)
        control.throttle=abs(axis3)/1.08
        if abs(axis2)>0.5:
            control.brake=1
        control.reverse=self._is_on_reverse
        # if keys[K_LEFT] or keys[K_a]:
        #     control.steer = -1.0
        # if keys[K_RIGHT] or keys[K_d]:
        #     control.steer = 1.0
        # if keys[K_UP] or keys[K_w]:
        #     control.throttle = 1.0
        # if keys[K_DOWN] or keys[K_s]:
        #     control.brake = 1.0
        # if keys[K_SPACE]:
        #     control.hand_brake = True
        # if keys[K_q]:
        #     self._is_on_reverse = not self._is_on_reverse
        # if keys[K_p]:
        #     self._enable_autopilot = not self._enable_autopilot
        # control.reverse = self._is_on_reverse
        return control

    def _print_player_measurements_map(
            self,
            player_measurements,
            map_position,
            lane_orientation):
        message = 'Step {step} ({fps:.1f} FPS): '
        message += 'Map Position ({map_x:.1f},{map_y:.1f}) '
        message += 'Lane Orientation ({ori_x:.1f},{ori_y:.1f}) '
        message += '{speed:.2f} km/h, '
        message += '{other_lane:.0f}% other lane, {offroad:.0f}% off-road'
        message = message.format(
            map_x=map_position[0],
            map_y=map_position[1],
            ori_x=lane_orientation[0],
            ori_y=lane_orientation[1],
            step=self._timer.step,
            fps=self._timer.ticks_per_second(),
            speed=player_measurements.forward_speed * 3.6,
            other_lane=100 * player_measurements.intersection_otherlane,
            offroad=100 * player_measurements.intersection_offroad)
        print_over_same_line(message)

    def _print_player_measurements(self, player_measurements):
        message = 'Step {step} ({fps:.1f} FPS): '
        message += '{speed:.2f} km/h, '
        message += '{other_lane:.0f}% other lane, {offroad:.0f}% off-road, '
        message += ' speed  : {throttle}, '
        message += ' acceleration : {steer}'
        message = message.format(
            step=self._timer.step,
            fps=self._timer.ticks_per_second(),
            speed=player_measurements.forward_speed * 3.6,
            other_lane=100 * player_measurements.intersection_otherlane,
            offroad=100 * player_measurements.intersection_offroad,
            steer=player_measurements.autopilot_control.steer,
            throttle=player_measurements.autopilot_control.throttle)
        print_over_same_line(message)

    def _on_render(self):
        gap_x = (WINDOW_WIDTH - 2 * MINI_WINDOW_WIDTH) / 3
        mini_image_y = WINDOW_HEIGHT - MINI_WINDOW_HEIGHT - gap_x

        if self._main_image is not None:
            array = image_converter.to_rgb_array(self._main_image)
            array = self.process_image(array)
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

            self._display.blit(surface, (0, 0))

        # if self._mini_view_image1 is not None:
        #     array = image_converter.depth_to_logarithmic_grayscale(self._mini_view_image1)
        #     surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        #     self._display.blit(surface, (gap_x, mini_image_y))
        #
        # if self._mini_view_image2 is not None:
        #     array = image_converter.labels_to_cityscapes_palette(
        #         self._mini_view_image2)
        #     surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

            # self._display.blit(
            #     surface, (2 * gap_x + MINI_WINDOW_WIDTH, mini_image_y))

        # if self._lidar_measurement is not None:
        #     lidar_data = np.array(self._lidar_measurement.data[:, :2])
        #     lidar_data *= 2.0
        #     lidar_data += 100.0
        #     lidar_data = np.fabs(lidar_data)
        #     lidar_data = lidar_data.astype(np.int32)
        #     lidar_data = np.reshape(lidar_data, (-1, 2))
        #     #draw lidar
        #     lidar_img_size = (200, 200, 3)
        #     lidar_img = np.zeros(lidar_img_size)
        #     lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
        #     surface = pygame.surfarray.make_surface(lidar_img)
        #     self._display.blit(surface, (10, 10))

        # if self._map_view is not None:
        #     array = self._map_view
        #     array = array[:, :, :3]
        #
        #     new_window_width = \
        #         (float(WINDOW_HEIGHT) / float(self._map_shape[0])) * \
        #         float(self._map_shape[1])
        #     surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        #
        #     w_pos = int(self._position[0]*(float(WINDOW_HEIGHT)/float(self._map_shape[0])))
        #     h_pos = int(self._position[1] *(new_window_width/float(self._map_shape[1])))
        #
        #     pygame.draw.circle(surface, [255, 0, 0, 255], (w_pos, h_pos), 6, 0)
        #     for agent in self._agent_positions:
        #         if agent.HasField('vehicle'):
        #             agent_position = self._map.convert_to_pixel([
        #                 agent.vehicle.transform.location.x,
        #                 agent.vehicle.transform.location.y,
        #                 agent.vehicle.transform.location.z])
        #
        #             w_pos = int(agent_position[0]*(float(WINDOW_HEIGHT)/float(self._map_shape[0])))
        #             h_pos = int(agent_position[1] *(new_window_width/float(self._map_shape[1])))
        #
        #             pygame.draw.circle(surface, [255, 0, 255, 255], (w_pos, h_pos), 4, 0)

            # self._display.blit(surface, (WINDOW_WIDTH, 0))

        pygame.display.flip()


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '-l', '--lidar',
        action='store_true',
        help='enable Lidar')
    argparser.add_argument(
        '-q', '--quality-level',
        choices=['Low', 'Epic'],
        type=lambda s: s.title(),
        default='Epic',
        help='graphics quality level, a lower level makes the simulation run considerably faster')
    argparser.add_argument(
        '-m', '--map',
        action='store_true',
        help='plot the map of the current city')
    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    while True:
        try:

            with make_carla_client(args.host, args.port) as client:
                game = CarlaGame(client, args)
                game.execute()
                break

        except TCPConnectionError as error:
            logging.error(error)
            time.sleep(1)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    name_dict['Image'] = imgg
    name_dict['Steer'] = st
    name_dict['throttle'] = th
    df = pd.DataFrame(name_dict)
    df.to_csv('Data.csv')
    print('\nCancelled by user. Bye!')
