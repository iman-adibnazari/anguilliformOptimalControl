#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
import numpy as np
from dotenv import dotenv_values 
import scipy.io
import cvxpy as cp
import logging 


class stateEstimator_LOpInf(): 
    def __init__(self,dt,logResults = False, logfile ='./stateEstimatorLog.log', *args, **kwargs):
        self.time = 0 # current time
        self.dt = dt # time step
        self.logResults = logResults
        # Read in system matrices and offset vectors
        systemMatFile = kwargs.get("systemMatFile")
        systemMats = scipy.io.loadmat(systemMatFile)
        self.A = systemMats['A_lopinf']
        self.B = systemMats['B_lopinf']
        self.C = systemMats['C_lopinf']
        self.D = systemMats['D_lopinf']
        self.L = systemMats['L_lopinf']
        # Initialize observer estimates
        self.u = np.zeros((self.B.shape[1],1))
        self.x_hat = np.zeros((self.A.shape[0],1))
        self.y_hat = np.zeros((self.C.shape[0],1))
        # simulation and optimization parameters
        self.n = 22 # number of states
        self.m = 6 # number of inputs
        self.p = 40 # number of outputs
        if self.logResults:
            logging.info('u: {}'.format(self.u.squeeze()))
            logging.info('x_hat: {}'.format(self.x_hat.squeeze()))
            logging.info('y_hat: {}'.format(self.y_hat.squeeze()))

    def updateState(self,u,y):
        # Reshape y to be a column vector
        y = y.reshape(-1,1)
        # Update state estimate
        self.u = np.reshape(u,(self.m,1))
        self.x_hat = self.A @ self.x_hat + self.B @ self.u - self.L @ (self.y_hat-y)
        self.y_hat = self.C @ self.x_hat
        if self.logResults:
            logging.info('u: {}'.format(self.u.squeeze()))
            logging.info('x_hat: {}'.format(self.x_hat.squeeze()))
            logging.info('y_hat: {}'.format(self.y_hat.squeeze()))
        return self.x_hat