
import datetime
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

GREEN_CONTOUR_AREA: float = 1_000.0
YELLOW_CONTOUR_AREA: float = 1_000.0

robot_has_object_in_arms: bool= False

robot = Pitop()
robot.add_component(DriveController(left_motor_port = "M0", right_motor_port = "M3"))
robot.add_component(Camera(rotate_angle = -90))
robot.add_component(PincerController(left_pincer_port="S3", right_pincer_port="S0"))
led_left = LED("D3")
led_right = LED("D4")
# buzzer = Buzzer("D0")


def go_to_green_object(frame):
    frame = ImageFunctions.convert(frame, format="OpenCV")

    image_mask = get_color_mask(frame, 'green')

    line_contour = find_largest_contour(image_mask)

    centroid = find_centroid(line_contour)

    if (centroid[0] is None or centroid[1] is None):
        robot.miniscreen.display_image(frame)
        robot.drive.left(0.05, 0)
        return

    display_frame = robot_view(frame, image_mask, line_contour, centroid)
    # robot.miniscreen.display_image(display_frame)

    contour_area = cv2.contourArea(line_contour)
    if (contour_area > GREEN_CONTOUR_AREA):
        robot.camera.capture_image(output_file_name=f"INTELEGENCE_COLLECTION_GREEN_SPHERE_{datetime.datetime.now()}.png")
        led_left.on()
        led_right.on()
        sleep(1.0)
        led_left.off()
        led_right.off()

    else:
        angle = get_object_target_lock_control_angle((centroid[0] - (frame.shape[1] / 2), centroid[1] - (frame.shape[0] / 2)), frame)

        if (abs(angle < MINOR_ANGLE_RANGE / 2)):
            robot.drive.forward(0.4)
            return

        TURN_SPEED: float = -(angle / 90) # / 2 # [-1, 1]
        DRIVE_SPEED: float = -abs(TURN_SPEED) + 1.0
        # print(f'{TURN_SPEED: .2f}, {DRIVE_SPEED: .2f}') 
        robot.drive.robot_move(DRIVE_SPEED * DRIVE_SPEED_FACTOR, TURN_SPEED * TURN_SPEED_FACTOR, robot.drive.wheel_separation / 2)

def main():
    '''
        ========== PHASE TWO (Part One) ==========
    '''

    SPEED: float = 0.5
    FORWARD_DELAY_SECONDS: float = 5.25

    # robot.drive.rotate(90, 10.0)

    robot.drive.forward(SPEED)
    sleep(FORWARD_DELAY_SECONDS)
    robot.drive.stop()

    sleep(0.5)

    robot.drive.right(0.125, 0)
    sleep(3.0)
    robot.drive.stop_rotation()
    
    robot.camera.capture_image(output_file_name=f"INTELEGENCE_COLLECTION_PLANE_{datetime.datetime.now()}.png")
    sleep(1.0)

    robot.drive.right(0.125, 0)
    sleep(2.5)
    robot.drive.stop_rotation()

    sleep(0.5)
 
    robot.drive.forward(SPEED)
    sleep(FORWARD_DELAY_SECONDS + 0.5)
    robot.drive.stop()

    sleep(0.5)

    robot.drive.left(0.125, 0)
    sleep(2.75)
    robot.drive.stop_rotation()
    

    '''
        ========== PHASE TWO (Part Two) ==========
    '''
    robot.camera.on_frame = go_to_green_object
    pause()

if __name__ == '__main__':
    main()