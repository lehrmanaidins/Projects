
'''
    Main Python File
    
    File: main.py
    Author: Aidin Lehrman
    Version: 2-26-2024 10:06
'''

import numpy as np
from signal import pause
from actions.line_detect import process_frame_for_line, robot_view
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

LARGE_TURN_SPEED_FACTOR: float = 0.025
SMALL_TURN_SPEED_FACTOR: float = 0.035
SLOW_DRIVE_SPEED_FACTOR: float = 0.15
FAST_DRIVE_SPEED_FACTOR: float = 0.3
MAJOR_ANGLE_RANGE: float = 10.0
MINOR_ANGLE_RANGE: float = 5.0
OBJECT_CLOSE_ARMS_AREA: float = 23_500.0
TURN_RADIUS: float = 3.0

COLOR_OF_OBJECT: str = 'red'

robot_has_object_in_arms: bool= False

robot = Pitop()
robot.add_component(DriveController(left_motor_port = "M0", right_motor_port = "M3"))
robot.add_component(Camera(rotate_angle = -90))
robot.add_component(PincerController(left_pincer_port="S3", right_pincer_port="S0"))
led_left = LED("D3")
led_right = LED("D4")
buzzer = Buzzer("D0")

def handle_frame_to_follow_line(frame):
    global turn_direction
    
    processed_frame = None

    crop_ratio: int = 2
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

        robot.drive.backward(SLOW_DRIVE_SPEED_FACTOR)
        sleep(0.25)
        robot.drive.stop()

    else:
        led_left.off()
        led_right.off()

        robot.miniscreen.display_image(processed_frame.robot_view)

        angle: float = processed_frame.angle

        if (abs(angle) > MINOR_ANGLE_RANGE / 2):
            robot.drive.forward(0.0)
            robot.drive.right((abs(angle) / angle) * LARGE_TURN_SPEED_FACTOR)
        else:
            robot.drive.stop_rotation()
            robot.drive.forward(SLOW_DRIVE_SPEED_FACTOR)

        # robot.miniscreen.display_text(f'{processed_frame.angle} deg.')

def get_color_mask(frame, color: str):
    blurred = cv2.blur(frame, (11, 11))
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    masks = []
    color_ranges = {
        "red": [
            {"lower": (150, 100, 100), "upper": (179, 255, 255)},
            {"lower": (0, 100, 100), "upper": (5, 255, 255)},
        ],
        "orange": [{"lower": (10, 100, 20), "upper": (25, 255, 255)}],
        "green": [{"lower": (60, 100, 100), "upper": (90, 255, 255)}],
        # "blue": [{"lower": (100, 100, 100), "upper": (130, 255, 255)}],
        "blue": [{"lower": (110, 150, 150), "upper": (130, 255, 255)}],
    }

    for color_range in color_ranges[color]:
        hsv_lower = color_range["lower"]
        hsv_upper = color_range["upper"]
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        masks.append(mask)
    mask = sum(masks)

    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)

    return mask


def main():
    robot.camera.on_frame = handle_frame_to_follow_line

if __name__ == '__main__':
    main() 
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

    pause()
