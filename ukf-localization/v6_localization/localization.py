from robot_UKF import RobotUKF
from numpy.linalg import norm
import numpy as np
import qi
import unboard
import logging
import time

session = qi.Session()

def main():

    PLOT = 1
    PLOT_STEP = 10
    FILENAME = "output.txt"

    # Arrays to store prediction and update data for later analysis
    x_prior_array = []  # Predicted state
    p_prior_array = []  # Predicted covariance
    x_post_array = []   # Updated state
    p_post_array = []   # Updated covariance
    dt_array = []       # Time steps between updates

    i = 0
    PRINT_STEP = 400

    # MAP
    field_map = { 
        64: np.array([[2.0, -0.5]]).T,  # Landmark 64 position
        108: np.array([[2.0, 0.5]]).T   # Landmark 108 position
    }

    logging.info("Creating RobotUKF object...")
    ukf = RobotUKF(dt=0.8, session=session)

    # Parameter initialization
    ukf.x = np.array([0, 0])                 # Initial robot state [x, y]
    ukf.P = np.diag([0.3, 0.3])              # Initial uncertainty covariance matrix
    ukf.Q = np.diag([0.3**2, 0.3**2])        # Process noise covariance
    ukf.R = np.diag([0.0825**2, 0.0825**2])  # Measurement noise covariance

    # Start localization
    unboard.is_calibrated = True
    prev_time = time.time()

    while unboard.run_localization:
        # PREDICTION STEP
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time
        ukf.predict(dt=dt)

        # Log prediction information at regular intervals
        if (i % PRINT_STEP) == 0:
            logging.debug("Prediction")
            logging.debug("x: %s", ukf.x[0])
            logging.debug("y: %s", ukf.x[1])
            logging.debug("P: %s", ukf.P[0, 0])
            logging.debug("P: %s", ukf.P[1, 1])

        # UPDATE STEP
        if not unboard.got_landmark:
            # If no landmark is detected, perform prediction-only update
            ukf.update(z=None)
        else:
            # If landmarks are detected, update the state using measurements
            detected_landmarks = unboard.landmarks
            try:
                for lmark in detected_landmarks:
                    lmark_id = lmark[0][0][0]
                    z = np.array([lmark[1][0], lmark[2][0]])  # Detected position
                    lmark_real_pos = field_map.get(lmark_id)  # Real landmark position
                    ukf.update(z, lmark_real_pos)             # Perform UKF update
                
                # If two landmarks are detected, calculate the midpoint
                if len(detected_landmarks) == 2:
                    lmark1_real_pos = field_map.get(detected_landmarks[0][0][0])
                    lmark2_real_pos = field_map.get(detected_landmarks[1][0][0])
                    
                    midpoint = (lmark1_real_pos + lmark2_real_pos) / 2
                    
                    # Adjust the robot's position to be relative to the midpoint
                    ukf.x[:2] = ukf.x[:2] - midpoint[:, 0]
                    
            except Exception:
                pass
        
        # Check uncertainties and save data
        std_dev_x = np.sqrt(ukf.P[0, 0])
        std_dev_y = np.sqrt(ukf.P[1, 1])

        if (i % PRINT_STEP) == 0 and std_dev_x <= 0.3 and std_dev_y <= 0.3:
            logging.debug("Saw landmark: %s", unboard.got_landmark)
            logging.debug("x: %s", ukf.x[0])
            logging.debug("y: %s", ukf.x[1])
            logging.debug("P: %s", ukf.P[0, 0])
            logging.debug("P: %s", ukf.P[1, 1])
            
            # Save robot's current state and covariance to shared variables
            unboard.X = ukf.x[0]
            unboard.Y = ukf.x[1]
            unboard.P = ukf.P

            # Store data for file output
            dt_array.append(dt)
            x_prior_array.append(ukf.x.copy())
            p_prior_array.append(ukf.P.copy())
            x_post_array.append(ukf.x.copy())
            p_post_array.append(ukf.P.copy())

        i += 1

        # Save all data to the output file
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

    logging.info("Final position: %s", ukf.x)
    logging.info("Final covariance: %s", ukf.P)

if __name__ == "__main__":
    main()
