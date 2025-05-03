#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
import numpy as np
from dotenv import dotenv_values 
import scipy.io
import cvxpy as cp
import logging
import time

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



class rhcPolicy_DMDc(): 
    def __init__(self,dt,T=50, *args, **kwargs):
        self.time = 0 # current time
        self.dt = dt # time step
        self.length = 1114.1947932504659 # mm
        # Read in system matrices and offset vectors
        systemMatFile = kwargs.get("systemMatFile")
        systemMats = scipy.io.loadmat(systemMatFile)
        self.A = systemMats['A_dmdc']
        self.B = systemMats['B_dmdc']
        self.C = systemMats['C_dmdc']
        # self.D = systemMats['D_dmdc']
        self.L = systemMats['L_dmdc']
        # Initialize ROM state estimate
        self.x_hat = np.zeros((self.A.shape[0],1))
        # Initialize optimization problem for rhc using ROM state as initial condition
        # simulation and optimization parameters
        self.n = self.A.shape[0] # number of states
        self.m = 6 # number of inputs
        self.p = 40 # number of outputs
        self.T = T # prediction horizon
        self.Q = np.eye(self.n) # state cost matrix
        self.R = np.eye(self.m) # input cost matrix
        self.P = np.eye(self.n) # terminal state cost matrix
        # Formulate optimization problem
        n = self.A.shape[0] # number of states
        m = 6 # number of inputs
        p = 40 # number of outputs
        # T = 50 # prediction horizon
        Q = np.eye(n) # state cost matrix
        R = np.eye(m) # input cost matrix
        P = np.eye(n) # terminal state cost matrix
        self.x = cp.Variable((n, T + 1))
        self.x0 = cp.Parameter(n)
        self.x0.value = np.zeros(n)
        self.u = cp.Variable((m, T+1))
        self.u0 = cp.Parameter(m)
        self.u0.value = np.zeros(m)
        self.du = cp.Variable((m, T))
        self.y = cp.Variable((p, T + 1))
        self.y_ref = cp.Parameter((p, T + 1))
        self.u_max = 0.1 # maximum input
        # Costs and constraints
        cost = 0
        constr = []
        constr+= [self.x[:, 0] == self.x0]
        constr+= [self.u[:, 0] == self.u0]
        constr+= [self.y[:, 0] == self.C @ self.x[:, 0]] #+ self.D @ self.u[:, 0]]

        for t in range(T):
            # Apply cost for output trajectory      
            # Only apply cost for odd output indices to penalize the z trajectory error
            cost += 0.7*cp.sum_squares(self.y[1:3:2,t+1]-self.y_ref[1:3:2,t+1])
            cost += 0.65*cp.sum_squares(self.y[3:7:2,t+1]-self.y_ref[3:7:2,t+1])
            cost += 0.65*cp.sum_squares(self.y[7:11:2,t+1]-self.y_ref[7:11:2,t+1])
            cost += 0.5*cp.sum_squares(self.y[11:15:2,t+1]-self.y_ref[11:15:2,t+1])
            cost += 0.4*cp.sum_squares(self.y[15:39:2,t+1]-self.y_ref[15:39:2,t+1])


            # # Regularize how far the x trajectory is from the origin
            # cost_era += 0.1*cp.sum_squares(y_era[0::2,t+1]-y_ref[0::2,t+1])

            # if t % 2 == 1:
            # cost_era += cp.sum_squares(y_era[:, t + 1]-y_ref[:,t+1])#+ cp.sum_squares(u[:, t])
            cost+= 1800*cp.sum_squares(self.du[:, t])
            cost += 1500*cp.sum_squares(self.u[:, t+1])
            constr += [self.x[:, t + 1] == self.A @ self.x[:, t] + self.B @ self.u[:, t+1], cp.norm(self.u[:, t+1], "inf") <= self.u_max]
            constr += [self.y[:, t + 1] == self.C @ self.x[:, t + 1]] #+ self.D @ self.u[:, t+1]]
            constr += [self.u[:, t+1] == self.u[:, t] + self.du[:, t]]

            # Apply antagonistic control constraint
            constr += [self.u[0,t+1]==-self.u[1,t+1]]
            constr += [self.u[2,t+1]==-self.u[3,t+1]]
            constr += [self.u[4,t+1]==-self.u[5,t+1]]

            # constraints to limit change in input
            # if t > 0:
            #     constr_era += [cp.norm(u_era[:, t] - u_era[:, t - 1],"inf") <= 0.1]
        # sums problem objectives and concatenates constraints.
        self.rhcOpt = cp.Problem(cp.Minimize(cost), constr)


    def getAction(self,x0, y_ref,u0):
        # set initial conditions for optimization
        self.x0.value = x0.squeeze()
        self.u0.value = u0.squeeze() # use previous control input as initial condition
        self.y_ref.value = y_ref
        # solve optimization problem
        start = time.time()
        self.rhcOpt.solve(solver='GUROBI', verbose=False) # verbose=True
        end = time.time()
        # get first control input
        controlInput = self.u.value[:,1]
        # logging.info("ControlOptimization")
        # logging.info('controlInput: {}'.format(controlInput.squeeze()))
        print('controlInput: {}'.format(controlInput.squeeze()))
        # logging.info('Optimization time: {}'.format(end-start))
        # update timestep
        self.time += self.dt
        return controlInput
