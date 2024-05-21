#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
import numpy as np
from dotenv import dotenv_values 

# Defines a policy that follows a phased sinusoidal trajectory

class sinusoidalPolicy(): 
    def __init__(self,dt,*args, **kwargs):
        self.dt = dt
        self.phase = kwargs.get("phase")
        self.amplitude = kwargs.get("amplitude")
        self.frequency = kwargs.get("frequency")



    def getAction(self,x,t): 
        pressure = self.amplitude*np.sin(2*np.pi*self.frequency*t + self.phase * np.pi/180)
        return pressure