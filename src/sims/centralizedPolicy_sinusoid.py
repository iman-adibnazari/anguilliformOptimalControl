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

class centralizedPolicy_sinusoid(): 
    def __init__(self,dt,T=50, *args, **kwargs):
        self.dt = dt # time step
        self.time = 0 # current time
        self.length = 1114.1947932504659 # mm
        self.amplitudes = np.array([0.0025,0.001,0.000]) # units of pressure
        self.frequencies = np.array([0.5,0.5,0.5]) # Hz
        self.phases = np.array([0,120,240]) # degrees
    def getAction(self,x0, y_ref,u0):
        # Compute control inputs as 6 phased sinusoids with every other element being negative the element before it
        controlInput = np.array([self.amplitudes[0]*np.sin(2*np.pi*self.frequencies[0]*self.time + self.phases[0] * np.pi/180),
                                -self.amplitudes[0]*np.sin(2*np.pi*self.frequencies[0]*self.time + self.phases[0] * np.pi/180),
                                self.amplitudes[1]*np.sin(2*np.pi*self.frequencies[1]*self.time + self.phases[1] * np.pi/180),
                                -self.amplitudes[1]*np.sin(2*np.pi*self.frequencies[1]*self.time + self.phases[1] * np.pi/180),
                                self.amplitudes[2]*np.sin(2*np.pi*self.frequencies[2]*self.time + self.phases[2] * np.pi/180),
                                -self.amplitudes[2]*np.sin(2*np.pi*self.frequencies[2]*self.time + self.phases[2] * np.pi/180)])
        # update timestep
        self.time += self.dt
        print("time = "+self.time.__str__())
        # Print pressures
        print("Control Input = "+controlInput.__str__())
        return controlInput
