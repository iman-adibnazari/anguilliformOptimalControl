#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
import numpy as np
from dotenv import dotenv_values 
import scipy.io
import cvxpy as cp
import logging
import time


class rhcPolicy_randomizedOutput(): 
    def __init__(self,dt,T=50, *args, **kwargs):
        self.time = 0 # current time
        self.dt = dt # time step
        self.length = 1114.1947932504659 # mm
        # Simulation and optimization parameters
        self.n = 22 # number of states
        self.m = 6 # number of inputs
        self.p = 40 # number of outputs
        self.T = T # prediction horizon
        self.u = np.zeros((self.m)) # initialize control input
        self.u_max = 0.15 # maximum control input
        # Random number generator that pulls gaussian random numbers from a distribution with mean 0 and std 0.1
        self.randomGenerator = np.random.default_rng()
        # Formulate optimization problem
        n = 22 # number of states
        m = 6 # number of inputs
        p = 40 # number of outputs

    def getAction(self,x0, y_ref,u0):
        # compute control inputs from random number generator
        start = time.time()
        du = self.randomGenerator.normal(0,0.005,(self.m))
        self.u = self.u+du
        # let u saturate
        self.u[self.u>self.u_max] = self.u_max
        self.u[self.u<-self.u_max] = -self.u_max
        end = time.time()
        # get first control input
        controlInput = self.u.squeeze()
        logging.info("ControlOptimization")
        logging.info('controlInput: {}'.format(controlInput.squeeze()))
        logging.info('Optimization time: {}'.format(end-start))
        # update timestep
        self.time += self.dt
        return controlInput
