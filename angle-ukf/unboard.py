import numpy as np

########################
#     Localization     #
########################

is_calibrated = False
P = np.diag([0.3, 0.3])
X = 0.0 
Y = 0.0

########################
#  Landmark Detector   #
########################

run_localization = True
got_landmark = False
landmarks = None