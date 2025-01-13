#!/usr/bin/env python

"""
Adapted from UnBeatables
"""

import qi
import argparse
import sys
import os
import threading

import logging
logging.basicConfig(level=logging.NOTSET)


# Adding code modules to library path
sys.path.append(os.path.join(sys.path[0], 'v6_action'))
sys.path.append(os.path.join(sys.path[0], 'v6_localization'))
sys.path.append(os.path.join(sys.path[0], 'v6_perception'))


# Importing code modules
import action
import localization
import landmark_detector


# Connect to qi Session
session = qi.Session()
try:
    session.connect("tcp://nao.local:9559") 
    logging.debug("Connected")

except RuntimeError:
    logging.exception("Can't connect to Naoqi")
    sys.exit(1)

action.session = session
localization.session = session
landmark_detector.session = session


try:
    logging.info("Starting action thread ...")
    action_thread = threading.Thread(target=action.main)
    action_thread.start()
    
    logging.info("Starting localization thread ...")
    localization_thread = threading.Thread(target=localization.main)
    localization_thread.start()

    logging.info("Starting landmark detector thread ...")
    landmark_detector_thread = threading.Thread(target=landmark_detector.main)
    landmark_detector_thread.start()

    action_thread.join()
    localization_thread.join()
    landmark_detector_thread.join()

except:
    sys.exit(1)