import qi
import time
import unboard
import numpy as np
from math import sqrt

session = qi.Session()

# Configure movement parameters
total_distance = 1.6  # Total distance to be traveled in meters
step_distance = 0.2   # Distance of each step in meters

def main():

    motion_service = session.service("ALMotion")
    motion_service.setStiffnesses("Body", 1.0)

    # Define the relative positions of the landmarks (L1 and L2)
    landmark_positions = {
        '1': np.array([0.0, -0.5]),  # L1 position relative to the midpoint
        '2': np.array([0.0, 0.5]),   # L2 position relative to the midpoint
    }

    # Loop to ensure valid input from the user
    while True:
        landmark_choice = raw_input("Which landmark? (1 or 2): ")

        if landmark_choice in landmark_positions:
            target_pos = landmark_positions[landmark_choice]  # Set the position of the chosen landmark
            break  
        else:
            print("Invalid choice")

    distance_moved = 0.0  # Total distance traveled
    use_midpoint = True   # Initially use the midpoint as the reference

    # Main navigation loop while localization is active and the distance is not reached
    while unboard.run_localization and distance_moved < total_distance:
        # Check if the system is calibrated and the covariance matrix is available
        if unboard.is_calibrated and unboard.P is not None:
            # Calculate the standard deviation of x and y positions
            std_dev_x = sqrt(unboard.P[0, 0])
            std_dev_y = sqrt(unboard.P[1, 1])
            
            # If the standard deviations are high, stop the movement to ensure higher precision
            if std_dev_x > 0.3 or std_dev_y > 0.3:
                motion_service.stopMove()
                time.sleep(1)
                continue
            else:
                # Get the robot's relative position
                relative_pos = np.array([unboard.X, unboard.Y])
                
                if use_midpoint:
                    # Calculate the difference between the current position and the landmark relative to the midpoint
                    diff = relative_pos - target_pos
                    
                    # Print the y position relative to the midpoint
                    print("Current y position relative to midpoint:", relative_pos[1])
                    
                    # Check if the reference should be switched to the landmark
                    if (landmark_choice == '1' and relative_pos[1] <= -0.5) or \
                       (landmark_choice == '2' and relative_pos[1] >= 0.5):
                        print("Switching to landmark reference")
                        use_midpoint = False
                        # Update the reference to the absolute position of the landmark
                        target_pos = np.array([2.0, -0.5]) if landmark_choice == '1' else np.array([2.0, 0.5])
                else:
                    # Calculate the difference directly relative to the chosen landmark
                    diff = relative_pos - target_pos
                    # Print the current distance to the landmark
                    print("Distance to target (landmark reference):", np.linalg.norm(diff))
                
                # Check the relative position and move the robot to adjust its position
                if diff[1] < -0.15:
                    motion_service.moveTo(step_distance, 0.15, 0)  # Move to the left
                    print("Moving left")
                    distance_moved += step_distance
                    time.sleep(2)
                    
                elif diff[1] > 0.15:
                    motion_service.moveTo(step_distance, -0.15, 0)  # Move to the right
                    print("Moving right")
                    distance_moved += step_distance
                    time.sleep(2)
                    
                else:
                    motion_service.moveTo(step_distance, 0, 0)  # Move forward
                    print("Moving forward")
                    distance_moved += step_distance
                    time.sleep(0.3)
        else:
            time.sleep(0.3)

    motion_service.stopMove()
    unboard.run_localization = False

if __name__ == "__main__":
    main()
