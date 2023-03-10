#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
import numpy as np
from dotenv import dotenv_values 

# Defines a policy that follows a discretized brownian stochastic process according to 
# https://sites.me.ucsb.edu/~moehlis/APC591/tutorials/tutorial7/node2.html

class brownianPolicy(): 
    def __init__(self,dt,*args, **kwargs):
        self.dt = dt
        self.seed = kwargs.get("seed") 
        self.internalState = 0
        self.variance = 1
        self.rng = np.random.default_rng(self.seed)
    def stepInternalState(self): 
        # self.internalState = np.max([np.min([self.internalState + np.sqrt(self.dt)*self.rng.normal(0,1),0.01]),-0.01])
        self.internalState = np.max([np.min([self.internalState + self.dt*self.rng.normal(0,1),0.025]),-0.025])

    def getAction(self,x,t): 
        # advance internal state of process 
        self.stepInternalState()
        # return action 
        return self.internalState