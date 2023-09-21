#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
import numpy as np
from dotenv import dotenv_values 
import scipy.io
import cvxpy as cp

# helper function to define reference trajectories
# Function to provide coordinates of discretized centerline at a given time
def generateReferenceCoords(time,numPoints=20,a_max=10,l=1114.1947932504659,k=10,omega=7,x_shift = 0,z_shift = 0):
    # Generate 10 times number of x coordinates as desired points
    x = np.linspace(0,l,numPoints*1000)
    dx = x[1]-x[0]
    # Compute integrand of arc length integral
    integrand = np.sqrt(1+(a_max/l*(np.exp(x/l-1)*(k*np.cos(k/l*x+omega*time)-np.sin(-k/l*x+omega*time))))**2)
    # Use first order quadrature to compute integral
    integrand = integrand*dx
    integrand = np.cumsum(integrand)
    # Find where the integral is closest to the desired arc length
    larc_des = np.linspace(0,l,numPoints)
    x_ref_0 = np.linspace(0,l,numPoints) # Initial x coordinate of each reference point
    x_ref = np.zeros(numPoints)
    z_ref = np.zeros(numPoints)
    for i in range(numPoints):
        idx = np.argmin(np.abs(integrand-larc_des[i]))
        x_ref[i] = x[idx]-x_ref_0[i] # We subtract off the initial x coordinate to make the first point at the origin
        # Compute z value at this point
        z_ref[i] = a_max*np.exp(x[idx]/l-1)*np.sin(k*x[idx]/l-omega*time)
    # z = z-z[0]
    x_ref = x_ref+x_shift
    z_ref = z_ref+z_shift
    y_ref = np.hstack((x_ref.reshape(-1,1),z_ref.reshape(-1,1))).reshape(-1,1).squeeze()
    return y_ref



class rhcPolicy_ERA(): 
    def __init__(self,dt,*args, **kwargs):
        self.time = 0 # current time
        self.dt = dt # time step
        self.length = 1114.1947932504659 # mm
        # Read in system matrices and offset vectors
        systemMatFile = kwargs.get("systemMatFile")
        systemMats = scipy.io.loadmat(systemMatFile)
        self.A = systemMats['A_era']
        self.B = systemMats['B_era']
        self.C = systemMats['C_era']
        self.D = systemMats['D_era']
        self.L = systemMats['L_era']
        # Initialize ROM state estimate
        self.x_hat = np.zeros((self.A.shape[0],1))
        # Initialize optimization problem for rhc using ROM state as initial condition
        # simulation and optimization parameters
        self.n = 22 # number of states
        self.m = 6 # number of inputs
        self.p = 40 # number of outputs
        self.T = 250 # prediction horizon
        self.Q = np.eye(self.n) # state cost matrix
        self.R = np.eye(self.m) # input cost matrix
        self.P = np.eye(self.n) # terminal state cost matrix

# TODO: Context - the era problem is correctly formulated in the notebook. Whats next is transferring that problem here. 
                # 1) copy the code from the notebook here for initializing the the problem and assign the parameters for x0,u0, and yref to variables so that they can be changed on the fly 
                # 2) figure out to pass the multiple control outputs from this module to the pressure controllers
                # 3) state estimation using last state, last pressure input and reduced centerline -> reparametrize optimization with state estimate as initial condition and new reference trajectory -> pass out control inputs to pressure controllers
                # 4) update timestep
                # 5) for debugging, export reduced centerlines somewhere and plot them in a notebook later to make sure they are correct
        self.rhcOpt = None 

    def stepInternalState(self): 
        # self.internalState = np.max([np.min([self.internalState + np.sqrt(self.dt)*self.rng.normal(0,1),0.01]),-0.01])
        self.internalState = np.max([np.min([self.internalState + self.dt*self.rng.normal(0,1),0.03]),-0.03])

    def getAction(self,x,t): 
        # advance internal state of process 
        self.stepInternalState()
        # return action 
        return self.internalState