
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

RED_CONTOUR_AREA: float = 12_500.0
ORANGE_CONTOUR_AREA: float = 12_500.0 #Gonna have to alter this depending on the orange area, probaby gonna be higher bc orange area is bigger

robot = Pitop()
robot.add_component(DriveController(left_motor_port = "M0", right_motor_port = "M3"))
robot.add_component(Camera(rotate_angle = -90))
robot.add_component(PincerController(left_pincer_port="S3", right_pincer_port="S0"))
led_left = LED("D3")
led_right = LED("D4")
# buzzer = Buzzer("D0")

def is_object_grabbable(frame):
    global robot_has_object_in_arms
    
    if (robot_has_object_in_arms):
        return 
    
    frame = ImageFunctions.convert(frame, format="OpenCV")

    image_mask = get_color_mask(frame, 'red')

    line_contour = find_largest_contour(image_mask)

    centroid = find_centroid(line_contour)

    if (centroid[0] is None or centroid[1] is None):
        robot.miniscreen.display_image(frame)
        robot.drive.right(0.05, 0)
        return

    display_frame = robot_view(frame, image_mask, line_contour, centroid)
    robot.miniscreen.display_image(display_frame)

    contour_area = cv2.contourArea(line_contour)
    if (contour_area > RED_CONTOUR_AREA):

        robot.pincers.open(speed = 50, angle = 60)
        robot.drive.backward(0.25)
        sleep(1.0)
        robot.drive.right(0.25) # Robot turns away as to not detect the red objects on orange block
        sleep(1.0) #Edit this time to get it at a 90 degree turn so it can look for more red
        robot.drove.stop_rotation()
        robot.camera.on_frame = is_object_grabbable

        # Close servos to grab object
        # Set some boolean value to true, which then makes the robot look for the oraneg block and drives towards that and opens arms.
        # repeat again
        # 


    else:
        angle = get_object_target_lock_control_angle((centroid[0] - (frame.shape[1] / 2), centroid[1] - (frame.shape[0] / 2)), frame)

        if (abs(angle < MINOR_ANGLE_RANGE / 2)):
            robot.drive.forward(0.4)
            return

        TURN_SPEED: float = -(angle / 90) # / 2 # [-1, 1]
        DRIVE_SPEED: float = -abs(TURN_SPEED) + 1.0
        # print(f'{TURN_SPEED: .2f}, {DRIVE_SPEED: .2f}') 
        robot.drive.robot_move(DRIVE_SPEED * DRIVE_SPEED_FACTOR, TURN_SPEED * TURN_SPEED_FACTOR, robot.drive.wheel_separation / 2)

def drive_to_orange():
    global robot_has_object_in_arms
    
    if (robot_has_object_in_arms):
        return 
    
    frame = ImageFunctions.convert(frame, format="OpenCV")

    image_mask = get_color_mask(frame, 'orange')

    line_contour = find_largest_contour(image_mask)

    centroid = find_centroid(line_contour)

    if (centroid[0] is None or centroid[1] is None):
        robot.miniscreen.display_image(frame)
        robot.drive.right(0.05, 0)
        return

    display_frame = robot_view(frame, image_mask, line_contour, centroid)
    robot.miniscreen.display_image(display_frame)

    contour_area = cv2.contourArea(line_contour)
    if (contour_area > ORANGE_CONTOUR_AREA):

        robot.pincers.open(speed = 50, angle = 60)
        robot.drive.backward(0.25)
        sleep(1.0)
        robot.drive.right(0.25) # Robot turns away as to not detect the red objects on orange block
        sleep(1.0) #Edit this time to get it at a 90 degree turn so it can look for more red
        robot.drove.stop_rotation()
        robot.camera.on_frame = is_object_grabbable

        # Close servos to grab object
        # Set some boolean value to true, which then makes the robot look for the oraneg block and drives towards that and opens arms.
        # repeat again
        # 
     

if __name__ == '__main__':
    robot.camera.on_frame = is_object_grabbable
    pause()

