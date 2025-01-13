from robot_UKF import RobotUKF
from numpy.linalg import norm
import numpy as np
import qi
import unboard
import logging
import time

session = qi.Session()

def main():
    # Settings for plotting and saving data
    PLOT = 1
    PLOT_STEP = 10
    FILENAME = "output.txt"
    
    # Arrays to store data
    x_prior_array = []
    p_prior_array = []
    x_post_array = []
    p_post_array = []
    dt_array = []

    # Logging variables
    i = 0
    PRINT_STEP = 400

    # MAP
    # Field map with known landmarks
    field_map = { 
        64: np.array([[2.0, -0.5]]).T, 
        108: np.array([[2.0, 0.5]]).T 
    }

    logging.info("Creating RobotUKF object...")
    ukf = RobotUKF(dt=0.8, session=session)

    # Parameter initialization
    # Initial state of the robot [x, y]
    ukf.x = np.array([0, 0])

    # Initial uncertainty covariance
    ukf.P = np.diag([0.3, 0.3])

    # Process uncertainty
    ukf.Q = np.diag([0.3**2, 0.3**2])

    # Measurement uncertainty
    ukf.R = np.diag([0.0825**2, 0.0825**2])

    # Localization loop
    unboard.is_calibrated = True
    prev_time = time.time()

    while unboard.run_localization:
        # PREDICTION
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time
        ukf.predict(dt=dt)

        # Debug logging
        if (i % PRINT_STEP) == 0:
            logging.debug("Prediction")
            logging.debug("x: %s", ukf.x[0])
            logging.debug("y: %s", ukf.x[1])
            logging.debug("P: %s", ukf.P[0, 0])
            logging.debug("P: %s", ukf.P[1, 1])

        # UPDATE
        if not unboard.got_landmark:
            ukf.update(z=None)
        else:
            detected_landmarks = unboard.landmarks
            try:
                for lmark in detected_landmarks:
                    lmark_id = lmark[0][0][0]
                    z = np.array([lmark[1][0], lmark[2][0]])
                    lmark_real_pos = field_map.get(lmark_id)
                    ukf.update(z, lmark_real_pos)
                
                # If two landmarks are detected, calculate the midpoint
                if len(detected_landmarks) == 2:
                    lmark1_real_pos = field_map.get(detected_landmarks[0][0][0])
                    lmark2_real_pos = field_map.get(detected_landmarks[1][0][0])
                    
                    # Calculate the midpoint between the two landmarks
                    midpoint = (lmark1_real_pos + lmark2_real_pos) / 2
                    
                    # Adjust the robot's position estimate to be relative to the midpoint
                    ukf.x[:2] = ukf.x[:2] - midpoint[:, 0]
                    
            except Exception:
                pass
        
        # Uncertainty check before saving data
        std_dev_x = np.sqrt(ukf.P[0, 0])
        std_dev_y = np.sqrt(ukf.P[1, 1])

        if (i % PRINT_STEP) == 0 and std_dev_x <= 0.3 and std_dev_y <= 0.3:
            logging.debug("Saw landmark: %s", unboard.got_landmark)
            logging.debug("x: %s", ukf.x[0])
            logging.debug("y: %s", ukf.x[1])
            logging.debug("P: %s", ukf.P[0, 0])
            logging.debug("P: %s", ukf.P[1, 1])
            
            unboard.X = ukf.x[0]
            unboard.Y = ukf.x[1]
            unboard.P = ukf.P

            # Save data to arrays for later output to file
            dt_array.append(dt)
            x_prior_array.append(ukf.x.copy())
            p_prior_array.append(ukf.P.copy())
            x_post_array.append(ukf.x.copy())
            p_post_array.append(ukf.P.copy())

        i += 1

        # Save data to file
        with open(FILENAME, "w") as f:
            for x_prior, p_prior, x_post, p_post, dt in zip(x_prior_array, p_prior_array, x_post_array, p_post_array, dt_array):
                f.write(str(dt))
                f.write(";")
                for x in x_prior:
                    f.write(str(x) + ",")
                f.write(";")
                for row in p_prior:
                    for p in row:
                        f.write(str(p) + ",")
                f.write(";")
                for x in x_post:
                    f.write(str(x) + ",")
                f.write(";")
                for row in p_post:
                    for p in row:
                        f.write(str(p) + ",")
                f.write("\n")

    # Final position
    logging.info("Final position: %s", ukf.x)
    logging.info("Final covariance: %s", ukf.P)

if __name__ == "__main__":
    main()
