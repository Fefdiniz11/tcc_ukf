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
        """ Transform state to measurement space 
        Parameters:
        - x: Robot's state [x, y].
        - lmark: Landmark's position [lx, ly].
        
        """
        
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
        
        """
        Parameters:
        - dt: Time step for prediction. Uses the default if None.
        - UT: Unscented transform function. Uses the default if None.
        """
        
        if dt is None:
            dt = self._dt

        if UT is None:
            UT = unscented_transform

        # Compute process sigma points
        self.compute_process_sigmas(dt, fx=self.fx)

        # Apply the unscented transform to predict the state and covariance
        self.x, self.P = UT(self.sigmas_f, self.Wm, self.Wc, self.Q, self.x_mean, self.residual_x)

        # Generate new sigma points
        self.sigmas_f = self.points_fn.sigma_points(self.x, self.P)

        # Save the prior state and covariance
        self.x_prior = np.copy(self.x)
        self.P_prior = np.copy(self.P)
        
        # Regularize the covariance matrix to ensure it is positive definite
        self.covariance_positive()
        
    def covariance_positive(self):
        
        # Ensure the covariance matrix is positive definite by adding small values to diagonal elements if necessary.
        min_eig = np.min(np.linalg.eigvals(self.P))
        if min_eig <= 0:
            self.P += np.eye(self.P.shape[0]) * (1e-10 - min_eig)
        
    def update(self, z, lmark_real_pos=None, R=None):

        """
        Parameters:
        - z: Measurement vector.
        - lmark_real_pos: Real position of the detected landmark.
        - R: Measurement noise covariance. Uses default if None.
        """

        # If no measurement is provided, retain the prior state and covariance
        if z is None:
            self.z = np.array([[None]*self._dim_z]).T
            self.x_post = self.x.copy()
            self.P_post = self.P.copy()
            return

        if R is None:
            R = self.R
        elif np.isscalar(R):
            R = np.eye(self._dim_z) * R         # Create a diagonal matrix if scalar is provided

        ## Transform sigma points to measurement space
        try:
            sigmas_h = []
            for s in self.sigmas_f:
                hx_result = self.hx(s, lmark_real_pos)
                assert hx_result.ndim == 1, hx_result.shape
                sigmas_h.append(hx_result)
            self.sigmas_h = np.atleast_2d(sigmas_h)     # matriz 2d

            # Apply the unscented transform to obtain the predicted measurement
            zp, self.S = unscented_transform(self.sigmas_h, self.Wm, self.Wc, R, self.z_mean, self.residual_z)
            self.SI = self.inv(self.S)              # Inverse of innovation covariance


            # Compute cross covariance
            Pxz = self.cross_variance(self.x, zp, self.sigmas_f, self.sigmas_h)

            # Compute Kalman gain
            self.K = dot(Pxz, self.SI)

            # Compute residual (measurement innovation)
            self.y = self.residual_z(z, zp)

            # Update state estimate with Kalman gain
            self.x = self.state_add(self.x, dot(self.K, self.y))
            self.P = self.P - dot(self.K, dot(self.S, self.K.T))

            # Regularize covariance matrix
            self.covariance_positive()

            # Save the measurement, updated state, and covariance
            self.z = deepcopy(z)
            self.x_post = self.x.copy()
            self.P_post = self.P.copy()
            
        except Exception as e:
            print("Erro ao calcular unscented_transform: ", e)
            traceback.print_exc()
            
    def cross_variance(self, x, z, sigmas_f, sigmas_h):
        
        """
        Parameters:
        - x: Predicted state mean.
        - z: Predicted measurement mean.
        - sigmas_f: Sigma points in state space.
        - sigmas_h: Sigma points in measurement space.

        Returns:
        - Pxz: Cross covariance matrix.
        """

        Pxz = np.zeros((sigmas_f.shape[1], sigmas_h.shape[1]))  # Cross covariance matrix
        N = sigmas_f.shape[0]   # Number of sigma points
        
        for i in range(N):
            dx = self.residual_x(sigmas_f[i], x)    # Difference in state space
            dz = self.residual_z(sigmas_h[i], z)    # Difference in measurement space
            Pxz += self.Wc[i] * np.outer(dx, dz)    # Weighted outer product
        return Pxz 
    
    def compute_process_sigmas(self, dt, fx=None):
        
        """
         Parameters:
        - dt: Time step.
        - fx: State transition function. Uses the default if None.
        """
        
        # Compute sigma points after applying the state transition function.
        if fx is None:
            fx = self.fx

        sigmas = self.points_fn.sigma_points(self.x, self.P)

        for i, s in enumerate(sigmas):
            self.sigmas_f[i] = fx(s, dt)

        # Ensure the covariance matrix remains positive definite
        self.covariance_positive()
    
    def state_add(self, x, dx):
        """
        Parameters:
        - x: Current state.
        - dx: Residual (correction).

        Returns:
        - Updated state.
        """
        
        # Correct the state estimate based on the measurement residual
        x[:2] += dx[:2]
        return x
