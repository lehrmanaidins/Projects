
'''
    Test Mover

    File: test_move.py
    Author: Aidin Lehrman
    Version: 1-23-2024 14:23
'''

from pitop.robotics.blockpi_rover import BlockPiRover
from time import sleep
import math

def main():
    robot = BlockPiRover()
    
    BASE_LENGTH = 1.0

    for _ in range(4):
        robot.forward(speed_factor = BASE_LENGTH)
        sleep(BASE_LENGTH)
        robot.left(speed_factor = BASE_LENGTH, turn_radius = BASE_LENGTH)
        sleep(BASE_LENGTH * math.pi)

    robot.stop()

if __name__ == '__main__':
    main()
