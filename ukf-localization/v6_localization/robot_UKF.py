# -*- coding: utf-8 -*-

from unscented_kalman_filter import UnscentedKalmanFilter as UKF
from unscented_transform import unscented_transform
from numpy import array, sqrt, dot
from math import sin, cos, tan, atan, asin, acos, atan2
import numpy as np
from numpy.random import rand
import scipy.linalg as linalg
from copy import deepcopy
import logging
from numpy.linalg import norm
import traceback

class RobotUKF(UKF):
    def __init__(self, dt, session):
        # Generates sigma points and weights according to Van der Merwe’s
        points = MerweScaledSigmaPoints(2, alpha=.1, beta=2., kappa=3-2)
        super(RobotUKF, self).__init__(dim_x=2, dim_z=2, dt=dt, fx=self.fx, hx=self.hx, points=points)
        
        # INITIALIZE SERVICES
        try:
            self.mem_service = session.service("ALMemory")  
            self.motion_service = session.service("ALMotion")
            self.motion_service.setStiffnesses("Body", 1.0)
            self.motion_service.moveInit()
        except Exception as e:
            logging.error("Error when creating services: %s", e)
            exit(1)
            
    def fx(self, x, dt):
        """ State transition function for the UKF """
        return x  
    
    def hx(self, x, lmark):
        """ Transform state to measurement space """
        
        x_pos = x[0]
        y_pos = x[1]
        
        lx = lmark[0]
        ly = lmark[1]
        
        distx = lx - x_pos
        disty = ly - y_pos
        
        coordinates = np.array([[distx, disty]]).T
        
        hx = coordinates
        
        return np.ravel(hx)     # array 1d

    def predict(self, dt=None, UT=None):
        if dt is None:
            dt = self._dt

        if UT is None:
            UT = unscented_transform

        self.compute_process_sigmas(dt, fx=self.fx)

        # Transformacao unscented
        self.x, self.P = UT(self.sigmas_f, self.Wm, self.Wc, self.Q, self.x_mean, self.residual_x)

        # novos pontos sigma
        self.sigmas_f = self.points_fn.sigma_points(self.x, self.P)

        self.x_prior = np.copy(self.x)
        self.P_prior = np.copy(self.P)
        
        # Regularizar a matriz de covariancia
        self.covariance_positive()
        
    def covariance_positive(self):
        # calcula os autovalores de uma matriz quadrada
        min_eig = np.min(np.linalg.eigvals(self.P))
        if min_eig <= 0:
            # para garantir que a matriz de covariancia seja positiva -> matriz identidade x pequeno valor positivo
            self.P += np.eye(self.P.shape[0]) * (1e-10 - min_eig)
        
    def update(self, z, lmark_real_pos=None, R=None):

        # se nao tiver medida, retorna os valores anteriores
        if z is None:
            self.z = np.array([[None]*self._dim_z]).T
            self.x_post = self.x.copy()
            self.P_post = self.P.copy()
            return

        if R is None:
            R = self.R
        elif np.isscalar(R):
            R = np.eye(self._dim_z) * R         # matriz diagonal

        # para cada ponto sigma, hx e aplicado p/ transformar o estado sigma no espaco de medicao
        try:
            sigmas_h = []
            for s in self.sigmas_f:
                hx_result = self.hx(s, lmark_real_pos)
                assert hx_result.ndim == 1, hx_result.shape
                sigmas_h.append(hx_result)
            self.sigmas_h = np.atleast_2d(sigmas_h)     # matriz 2d

            # UT aos pontos sigma transformados p/ obter a previsão de medicao zp
            zp, self.S = unscented_transform(self.sigmas_h, self.Wm, self.Wc, R, self.z_mean, self.residual_z)
            # inversa da matriz de covariancia da inovação pyy
            self.SI = self.inv(self.S)

            # covariancia cruzada
            Pxz = self.cross_variance(self.x, zp, self.sigmas_f, self.sigmas_h)

            # ganho de kalman Pxy*Pyy⁻1
            self.K = dot(Pxz, self.SI)

            # residuo - diferenca entre a medida e a medida prevista
            self.y = self.residual_z(z, zp)

            # atualizacao com o ganho de kalman
            self.x = self.state_add(self.x, dot(self.K, self.y))
            self.P = self.P - dot(self.K, dot(self.S, self.K.T))

            # regularizar a matriz de covariancia
            self.covariance_positive()

            self.z = deepcopy(z)
            self.x_post = self.x.copy()
            self.P_post = self.P.copy()
            
        except Exception as e:
            print("Erro ao calcular unscented_transform: ", e)
            traceback.print_exc()
            
    def cross_variance(self, x, z, sigmas_f, sigmas_h):

        # matriz de covariancia cruzada
        Pxz = np.zeros((sigmas_f.shape[1], sigmas_h.shape[1]))
        # pontos sigma
        N = sigmas_f.shape[0]
        for i in range(N):
            # diferenca entre sigma do estado previsto e o estado
            dx = self.residual_x(sigmas_f[i], x)
            # diferenca entre sigma da medicao prevista e a medicao
            dz = self.residual_z(sigmas_h[i], z)
            # produto externo ponderado pelo peso 
            Pxz += self.Wc[i] * np.outer(dx, dz)
        return Pxz
    
    def compute_process_sigmas(self, dt, fx=None):
        if fx is None:
            fx = self.fx

        sigmas = self.points_fn.sigma_points(self.x, self.P)

        # aplica a funcao de estado
        for i, s in enumerate(sigmas):
            self.sigmas_f[i] = fx(s, dt)

        # garantir que P seja positiva definida
        self.covariance_positive()
    
    def state_add(self, x, dx):
        # corrige a estimativa do estado com base nas medicoes
        x[:2] += dx[:2]
        return x
