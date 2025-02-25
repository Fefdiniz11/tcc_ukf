#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Adaptado da Softbank Robotics, com adição do stand init utilizando ALRobotPosture.
Este código é destinado ao Python 2.7 e executa no robô NAO.
"""

import qi
import time
import sys
import math
import almath
import logging
import numpy as np

import unboard


class LandmarkDetector(object):


    def __init__(self, session):
        
        super(LandmarkDetector, self).__init__()

        self.landmarkTheoreticalSize = 0.145

        self.currentCamera = "CameraTop"

        self.mem_service = session.service("ALMemory")

        self.landmark_detection = session.service("ALLandMarkDetection")
        self.motion_service = session.service("ALMotion")
        self.landmark_detection.subscribe("LandmarkDetector", 500, 0.0)

        self.mem_subscriber = self.mem_service.subscriber("LandmarkDetected")
        self.mem_subscriber.signal.connect(self.on_landmark_detected)

    def on_landmark_detected(self, markData):

        if markData == []:
            unboard.got_landmark = False
            unboard.landmarks = None
        else:
            unboard.got_landmark = True

            mark_info_array = markData[1]
            mark_pos_array = []

            for mark_info in mark_info_array:

                landmark_id = mark_info[1]
                if isinstance(landmark_id, list):
                    landmark_id = int(landmark_id[0])

                wzCamera = mark_info[0][1]
                wyCamera = mark_info[0][2]

                angularSize = mark_info[0][3]

                # Calcula a distância da câmera até a landmark utilizando semelhança de triângulos.
                distanceFromCameraToLandmark = self.landmarkTheoreticalSize / (2 * math.tan(angularSize / 2))

                # Calcula a transformação de rotação da câmera para a landmark.
                cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, wyCamera, wzCamera)

                # Calcula a transformação de translação para alcançar a landmark.
                cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)

                # Obtém a transformação da câmera atual para o espaço do robô.
                transform = self.motion_service.getTransform(self.currentCamera, 2, True)
                transformList = almath.vectorFloat(transform)
                robotToCamera = almath.Transform(transformList)

                # Combina as transformações para obter a posição da landmark no espaço do robô.
                robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * cameraToLandmarkTranslationTransform

                # Extrai as coordenadas (x, y) da landmark.
                landmark_x = robotToLandmark.r1_c4
                landmark_y = robotToLandmark.r2_c4

                # Extrai os ângulos de Euler (roll, pitch, yaw) da transformação (em radianos).
                roll, pitch, yaw = self.getEulerAngles(robotToLandmark)

                # Converte os ângulos para graus
                roll_deg = math.degrees(roll)
                pitch_deg = math.degrees(pitch)
                yaw_deg = math.degrees(yaw)

                # print "Landmark ID: %s, Position: (%.3f, %.3f), Orientation: roll=%.3f°, pitch=%.3f°, yaw=%.3f°" % (
                #     landmark_id, landmark_x, landmark_y, roll_deg, pitch_deg, yaw_deg)

                # Cria um array com os dados da landmark.
                mark_pos = np.array([[landmark_id, landmark_x, landmark_y, roll, pitch, yaw]])
                mark_pos_array.append(mark_pos)

            # Armazena as informações processadas no módulo unboard.
            unboard.landmarks = mark_pos_array

    def getEulerAngles(self, transform):
 
        # Extrai os elementos da matriz de rotação:
        r11 = transform.r1_c1
        r12 = transform.r1_c2
        r13 = transform.r1_c3
        r21 = transform.r2_c1
        r22 = transform.r2_c2
        r23 = transform.r2_c3
        r31 = transform.r3_c1
        r32 = transform.r3_c2
        r33 = transform.r3_c3

        # Cálculo do yaw (rotaciona em torno do eixo Z):
        yaw = math.atan2(r21, r11)
        # Cálculo do pitch (rotaciona em torno do eixo Y):
        pitch = math.atan2(-r31, math.sqrt(r32 * r32 + r33 * r33))
        # Cálculo do roll (rotaciona em torno do eixo X):
        roll = math.atan2(r32, r33)

        return roll, pitch, yaw

    def run(self):
        """
        Loop on, wait for events until manual interruption.
        Manual interruption only works when running this module alone.
        """
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print "Interrupted by user, stopping LandmarkDetector"
            self.landmark_detection.unsubscribe("LandmarkDetector")
            sys.exit(0)


session = qi.Session()

def main():
    landmark_detector = LandmarkDetector(session)
    landmark_detector.run()

if __name__ == "__main__":
    main()
