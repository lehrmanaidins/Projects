
'''
    Main Python File
    
    File: phase_one_line_following_script.py
    Author: Aidin Lehrman
    Version: 2-26-2024 10:06
'''

import datetime
import numpy as np
from signal import pause
from actions.line_detect import process_frame_for_line, robot_view
from actions.color_functions import get_color_mask

from pitop.camera import Camera
from pitop.core import ImageFunctions
from pitop.pma import LED, Buzzer
from pitop.robotics import BlockPiRover, DriveController, PincerController
from pitop import Pitop
from pitop.core.import_opencv import import_opencv
from pitop.processing.core.vision_functions import (
    color_mask,
    find_centroid,
    find_largest_contour,
    get_object_target_lock_control_angle,
)

from time import sleep
import math

cv2 = import_opencv()

DRIVE_SPEED_FACTOR: float = 1 / 15
TURN_SPEED_FACTOR: float = 4
MINOR_ANGLE_RANGE: float = 2.0
OBJECT_CLOSE_ARMS_AREA: float = 23_500.0

desired_left_motor_speed: float = 0.0
desired_right_motor_speed: float = 0.0

COLOR_OF_OBJECT: str = 'red'

robot_has_object_in_arms: bool= False

robot = Pitop()
robot.add_component(DriveController(left_motor_port = "M0", right_motor_port = "M3"))
robot.add_component(Camera(rotate_angle = -90))
robot.add_component(PincerController(left_pincer_port="S3", right_pincer_port="S0"))
led_left = LED("D3")
led_right = LED("D4")
# buzzer = Buzzer("D0")

def handle_frame_to_follow_line(frame):
    global turn_direction
    
    processed_frame = None

    crop_ratio: int = 4.5
    processed_frame = process_frame_for_line(frame, crop_ratio = crop_ratio)

    '''
    x1 = processed_frame.rectangle_dimensions[0][0]
    y1 = processed_frame.rectangle_dimensions[0][1]
    x2 = processed_frame.rectangle_dimensions[1][0]
    y2 = processed_frame.rectangle_dimensions[1][1]
    x3 = processed_frame.rectangle_dimensions[2][0]
    y3 = processed_frame.rectangle_dimensions[2][1]

    length: float = math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
    width: float = math.sqrt( (x2 - x3)**2 + (y2 - y3)**2 )
    area_of_rectangle: float = length * width
    '''
    # MIN_AREA_BLUE: float = 1500.0
    if (processed_frame.line_center is None): # or (area_of_rectangle < MIN_AREA_BLUE):
        led_left.on()
        led_right.on()
        
        robot.miniscreen.display_image(frame)

        robot.drive.stop_rotation()
        robot.drive.backward(0.1)
        sleep(0.25)
        robot.drive.stop()

    else:
        led_left.off()
        led_right.off()

        robot.miniscreen.display_image(processed_frame.robot_view)

        angle: float = processed_frame.angle
        
        if (abs(angle < MINOR_ANGLE_RANGE / 2)):
            robot.drive.forward(0.4)
            return

        TURN_SPEED: float = -(angle / 90) # / 2 # [-1, 1]
        DRIVE_SPEED: float = -abs(TURN_SPEED) + 1.0
        # print(f'{TURN_SPEED: .2f}, {DRIVE_SPEED: .2f}') 
        robot.drive.robot_move(DRIVE_SPEED * DRIVE_SPEED_FACTOR, TURN_SPEED * TURN_SPEED_FACTOR, robot.drive.wheel_separation / 2)

        # robot.miniscreen.display_text(f'{processed_frame.angle} deg.')


def phase_one_line_following_script():
    robot.camera.on_frame = handle_frame_to_follow_line

if __name__ == '__main__':

    # FAST_DRIVE_SPEED_FACTOR = 0.5
    # SLOW_DRIVE_SPEED_FACTOR = 0.2
    # sleep(10.0)


    '''
    led_left.off()
    led_right.off()
    robot.pincers.close(speed = 50, angle = 60)
    sleep(2.0)
    led_left.on()
    led_right.on()
    robot.pincers.open(speed = 50, angle = 90)
    sleep(2.0)
    '''
    
    # robot.pincers.open(speed = 50, angle = 90)
    # robot.camera.on_frame = is_object_grabbable
    
    '''
    while (True):
        led_left.off()
        led_right.off()
        robot.pincers.open(speed = 50, angle = 60)
        sleep(2.0)
        led_left.on()
        led_right.on()
        robot.pincers.close(speed = 50, angle = 0)
        sleep(2.0)
    '''

    ''''
        ========== PHASE ONE ==========
    '''
    
    phase_one_line_following_script()
    pause()

    sleep(50.0)
    DRIVE_SPEED_FACTOR = 1 / 25 # Slows speed to conquer the steep down slope of the first phase. 
    TURN_SPEED_FACTOR = 8

    robot.drive.right(0.125, 0)
    sleep(2.5)
    robot.drive.stop_rotation()

    robot.drive.forward(0.5)
    sleep(5.0)
    robot.drive.stop()

    DRIVE_SPEED_FACTOR = 0
    TURN_SPEED_FACTOR = 0

    while True:
        led_left.on()
        led_right.on()
        sleep(0.05)
        led_left.off()
        led_right.off()
        sleep(0.05)
