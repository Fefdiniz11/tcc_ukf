#! /usr/bin/env python
# -*- coding: utf-8 -*-

from robot_UKF import RobotUKF
from numpy.linalg import norm
import numpy as np
import qi
import unboard
import logging
import time

# Conecta à sessão
session = qi.Session()

def main():
    PLOT = 1
    PLOT_STEP = 10
    FILENAME = "output.txt"

    i = 0
    PRINT_STEP = 400

    field_map = { 
        64: np.array([[2.0], [0.0], [0.0]]),  
        108: np.array([[1.5], [2.0], [4.7]])  
    }

    logging.info("Criando objeto RobotUKF...")
    ukf = RobotUKF(dt=0.8, session=session)

    ukf.x = np.array([0, 0, 0])
    ukf.P = np.diag([0.4, 0.4, 0.4])
    ukf.Q = np.diag([0.4**2, 0.4**2, 0.4**2])
    ukf.R = np.diag([0.08**2, 0.08**2, 0.08**2])

    unboard.is_calibrated = True
    prev_time = time.time()

    while unboard.run_localization:
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time
        
        # print("dt:", dt)

        ukf.predict(dt=dt)

        # ETAPA DE UPDATE:
        if not unboard.got_landmark:
            ukf.update(z=None)
        else:
            detected_landmarks = unboard.landmarks
            try:
                for lmark in detected_landmarks:
                    lmark_id = lmark[0][0]  # ID da landmark
                    z = np.array([lmark[0][1], lmark[0][2], lmark[0][5]])
                    lmark_real_pos = field_map.get(lmark_id)
                    ukf.update(z, lmark_real_pos)
            except Exception as e:
                logging.error("Erro no update das landmarks: %s", e)
                pass

    ukf.final_state()

if __name__ == "__main__":
    main()
