import qi
import time
import unboard
import numpy as np
from math import sqrt

session = qi.Session()

total_distance = 1.6
step_distance = 0.2

def main():
    motion_service = session.service("ALMotion")
    motion_service.setStiffnesses("Body", 1.0)

    # Define the relative positions for the landmarks
    landmark_positions = {
        '1': np.array([0.0, -0.5]),  # L1 position relative to the midpoint
        '2': np.array([0.0, 0.5]),   # L2 position relative to the midpoint
    }

    # Loop to ensure valid input is provided
    while True:
        landmark_choice = raw_input("Which landmark? (1 or 2): ")

        if landmark_choice in landmark_positions:
            target_pos = landmark_positions[landmark_choice]
            break  
        else:
            print("Invalid choice")

    distance_moved = 0.0
    use_midpoint = True  # Initially use the midpoint for navigation

    while unboard.run_localization and distance_moved < total_distance:
        if unboard.is_calibrated and unboard.P is not None:
           
            std_dev_x = sqrt(unboard.P[0, 0])
            std_dev_y = sqrt(unboard.P[1, 1])
            if std_dev_x > 0.3 or std_dev_y > 0.3:
                motion_service.stopMove()
                time.sleep(1)
                continue
            else:
                # Check robot's relative position to the midpoint or target landmark
                relative_pos = np.array([unboard.X, unboard.Y])
                
                if use_midpoint:
                    # Calculate the difference to the target position relative to the midpoint
                    diff = relative_pos - target_pos
                    
                    # Print current position and check if y is close enough to switch reference
                    print("Current y position relative to midpoint:", relative_pos[1])
                    
                    if (landmark_choice == '1' and relative_pos[1] <= -0.5) or \
                       (landmark_choice == '2' and relative_pos[1] >= 0.5):
                        print("Switching to landmark reference")
                        use_midpoint = False
                        target_pos = np.array([2.0, -0.5]) if landmark_choice == '1' else np.array([2.0, 0.5])
                else:
                    # Calculate the difference to the target landmark directly
                    diff = relative_pos - target_pos
                    # Print distance using the landmark as the reference
                    print("Distance to target (landmark reference):", np.linalg.norm(diff))
                
                if diff[1] < -0.15:
                    motion_service.moveTo(step_distance, 0.15, 0)
                    print("Moving left")
                    distance_moved += step_distance
                    time.sleep(2)
                    
                elif diff[1] > 0.15:
                    motion_service.moveTo(step_distance, -0.15, 0)
                    print("Moving right")
                    distance_moved += step_distance
                    time.sleep(2)
                    
                else:
                    motion_service.moveTo(step_distance, 0, 0)
                    print("Moving forward")
                    distance_moved += step_distance
                    time.sleep(0.3)
        else:
            time.sleep(0.3)

    motion_service.stopMove()
    unboard.run_localization = False

if __name__ == "__main__":
    main()
