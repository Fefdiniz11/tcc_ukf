# -*- coding: utf-8 -*-

"""Copyright 2020 Débora Ferreira dos Santos

This is licensed under an MIT license.
"""

import time
import qi
from naoqi import ALProxy
import unboard
from robot_UKF import RobotUKF
import numpy as np
import logging

session = qi.Session()

zigzag = 0
square = 0
straight_line = 0
rectangle = 0
turn_around = 0
move_landmarks = 1

def main():
    motion_service = session.service("ALMotion")
    motion_service.setStiffnesses("Body", 1.0)
    
    logging.info("Criando objeto RobotUKF...")
    ukf = RobotUKF(dt=0.8, session=session)
    
    if not hasattr(unboard, "head_raised"):
        unboard.head_raised = False

    while unboard.run_localization:
        if unboard.is_calibrated:

            if not unboard.head_raised:
                motion_service.angleInterpolation("HeadPitch", -0.2, 1.0, True)
                unboard.head_raised = True

            if unboard.got_landmark:
                if move_landmarks:
                    
                    while True:
                        pos_std_x = np.sqrt(unboard.P[0, 0])
                        pos_std_y = np.sqrt(unboard.P[1, 1])
                        print(pos_std_x)
                        if pos_std_x < 0.6 and pos_std_y < 0.6:
                            break
                        time.sleep(0.1)
                    
                    step_distance = 0.5
                    total_distance = 1.5
                    steps = int(total_distance / step_distance)
                    for _ in range(steps):
                        motion_service.moveTo(step_distance, 0.0, 0.0)
                        time.sleep(0.5)
                    
                    motion_service.moveTo(0.0, 0.0, -1.8)
                    time.sleep(0.5)
                    
                    for _ in range(steps):
                        motion_service.moveTo(step_distance, 0.0, 0.0)
                        time.sleep(0.5)
                    
                    unboard.run_localization = False

            else:
                time.sleep(0.5)
        else:
            time.sleep(0.5)

if __name__ == "__main__":
    main()
