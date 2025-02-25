# -*- coding: utf-8 -*-

from unscented_kalman_filter import UnscentedKalmanFilter as UKF
from unscented_transform import unscented_transform
import numpy as np
import math
import logging
import traceback
from filterpy.kalman import MerweScaledSigmaPoints
from copy import deepcopy
import unboard

class RobotUKF(UKF):
    def __init__(self, dt, session):

        points = MerweScaledSigmaPoints(3, alpha=0.1, beta=2., kappa=3-3)
        super(RobotUKF, self).__init__(dim_x=3, dim_z=3, dt=dt, fx=self.fx, hx=self.hx, points=points)

        try:
            self.mem_service = session.service("ALMemory")
            self.motion_service = session.service("ALMotion")
            self.motion_service.setStiffnesses("Body", 1.0)
            self.motion_service.moveInit()
        except Exception as e:
            logging.error("Error when creating services: %s", e)
            exit(1)

        self.v = 0.5      
        self.omega = 1.0  

        
        self.predictions_since_update = 0 
        self.max_predictions = 3           

        self.iteration_counter = 0  
        self.save_interval = 10000  

    def fx(self, x, dt):
        
       
        return x

    def hx(self, x, lmark):
        
        xr, yr, theta = x[0], x[1], x[2]
        lx, ly, ltheta = lmark[0], lmark[1], lmark[2]
        
        dx = lx - xr
        dy = ly - yr
      
        x_rel = math.cos(theta) * dx + math.sin(theta) * dy
        y_rel = -math.sin(theta) * dx + math.cos(theta) * dy
        
        theta_rel = ltheta - theta
        theta_rel = (theta_rel + math.pi) % (2 * math.pi) - math.pi
        
        return np.array([x_rel, y_rel, theta_rel]).flatten()
    
    def log_state(self, phase):
        
        self.iteration_counter += 1

        if self.iteration_counter % self.save_interval != 0:
            return

        pos_std_x = np.sqrt(self.P[0, 0])
        pos_std_y = np.sqrt(self.P[1, 1])
        print(pos_std_x)

        if pos_std_x < 0.6 and pos_std_y < 0.6:
            theta_deg = math.degrees(self.x[2])
            if theta_deg < 0:
                theta_deg += 360
            state_in_degrees = np.array([self.x[0], self.x[1], theta_deg])

            with open("robot_states.txt", "a") as f:
                f.write("=== {} ===\n".format(phase))
                f.write("Iteration: {}\n".format(self.iteration_counter))
                f.write("State (x) in degrees: {}\n".format(state_in_degrees))
                f.write("Covariance (P):\n{}\n".format(self.P))
                f.write("--------------------------------------------------\n")

        print("Salvando estado após {} iterações.".format(self.iteration_counter))

    def predict(self, dt=None, UT=unscented_transform):  
        if dt is None:
            dt = self._dt

        
        if abs(self.x[0]) > 2.5 or abs(self.x[1]) > 2.5:
            print("Limite de posição ultrapassado! Parando o robô.")
            self.motion_service.stopMove()  
            unboard.run_localization = False 
            return 

        if self.predictions_since_update >= self.max_predictions:
            return

        self.compute_process_sigmas(dt, fx=self.fx)

        if UT is None:
            raise ValueError("Erro: UT está None. Verifique a chamada de predict().")

        self.x, self.P = UT(self.sigmas_f, self.Wm, self.Wc, self.Q, self.x_mean, self.residual_x)
        self.sigmas_f = self.points_fn.sigma_points(self.x, self.P)
        self.x_prior = np.copy(self.x)
        self.P_prior = np.copy(self.P)
        self.covariance_positive()

        self.log_state("PREDICTION")  
        self.predictions_since_update += 1



    
    def covariance_positive(self):

        min_eig = np.min(np.linalg.eigvals(self.P))
        if min_eig <= 0:
            self.P += np.eye(self.P.shape[0]) * (1e-10 - min_eig)
    
    def update(self, z, lmark_real_pos=None, R=None):
        if z is None:
            self.z = np.array([[None] * self._dim_z]).T
            self.x_post = self.x.copy()
            self.P_post = self.P.copy()
            return

        if R is None:
            R = self.R
        elif np.isscalar(R):
            R = np.eye(self._dim_z) * R

        try:
            sigmas_h = []
            for s in self.sigmas_f:
                hx_result = self.hx(s, lmark_real_pos)
                hx_result = np.array(hx_result).flatten()
                assert hx_result.ndim == 1, hx_result.shape
                sigmas_h.append(hx_result)
            self.sigmas_h = np.atleast_2d(sigmas_h)

            zp, self.S = unscented_transform(self.sigmas_h, self.Wm, self.Wc, R, self.z_mean, self.residual_z)
            self.SI = self.inv(self.S)
            Pxz = self.cross_variance(self.x, zp, self.sigmas_f, self.sigmas_h)
            self.K = np.dot(Pxz, self.SI)
            self.y = self.residual_z(z, zp)
            self.x = self.state_add(self.x, np.dot(self.K, self.y))

            self.P = self.P - np.dot(self.K, np.dot(self.S, self.K.T))
            self.covariance_positive()
            self.z = deepcopy(z)
            self.x_post = self.x.copy()
            self.P_post = self.P.copy()

            self.log_state("UPDATE")
            unboard.P = self.P.copy()
            
            std_x = np.sqrt(self.P[0, 0])
            std_y = np.sqrt(self.P[1, 1])
            print("Desvio padrão: x: {:.2f} m, y: {:.2f} m".format(std_x, std_y))

            self.predictions_since_update = 0 

        except Exception as e:
            print("Erro ao calcular unscented_transform:", e)
            traceback.print_exc()

    
    def cross_variance(self, x, z, sigmas_f, sigmas_h):
        
        Pxz = np.zeros((sigmas_f.shape[1], sigmas_h.shape[1]))
        N = sigmas_f.shape[0]
        for i in range(N):
            dx = self.residual_x(sigmas_f[i], x)
            dz = self.residual_z(sigmas_h[i], z)
            Pxz += self.Wc[i] * np.outer(dx, dz)
        return Pxz
    
    def compute_process_sigmas(self, dt, fx=None):

        if fx is None:
            fx = self.fx
        sigmas = self.points_fn.sigma_points(self.x, self.P)
        for i, s in enumerate(sigmas):
            self.sigmas_f[i] = fx(s, dt)
        self.covariance_positive()
    
    def state_add(self, x, dx):

        x[0] += dx[0]
        x[1] += dx[1]
        x[2] += dx[2]
        x[2] = (x[2] + math.pi) % (2 * math.pi) - math.pi
        return x
    
    def final_state(self):

        with open("robot_states.txt", "a") as f:
            theta_deg = math.degrees(self.x[2])
            if theta_deg < 0:
                theta_deg += 360
            state_in_degrees = np.array([self.x[0], self.x[1], theta_deg])
            f.write("=== FINAL STATE ===\n")
            f.write("State (x) in degrees: {}\n".format(state_in_degrees))
            f.write("Covariance (P):\n{}\n".format(self.P))
            f.write("==================================================\n")
        print("Final Robot State (in degrees): x: {:.2f}, y: {:.2f}, theta: {:.2f}°".format(self.x[0], self.x[1], theta_deg))
