#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key
import numpy as np
from dotenv import dotenv_values 

config = dotenv_values(".env")
''' 
deltaT - timestep
policy - function handle for the control policy p(x,t) where x is the full state vector and t is time
saveOutput - 0 to not save the outputs and 1 to save the outputs in npy files
'''
class PressureConstraintController(Sofa.Core.Controller):
    def __init__(self,deltaT, policy, saveOutput, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.step_id = 0 
        self.deltaT = deltaT
        self.node = args[0] 
        self.policy=policy
        self.saveOutput=saveOutput
        self.pressureConstraint = self.node.getObject('SurfacePressureConstraint')
        self.pressureConstraint.value = [0] 
        print(self.name.getValueString().__str__())

    def onAnimateBeginEvent(self, e):
        t=self.step_id*self.deltaT # get current time step
        
        self.pressureConstraint.value = [self.policy(0,t)]
        if self.saveOutput==1: 
            dataArray = np.array([t,self.pressureConstraint.value[0]]) 
            filename = config["currentDirectory"]+"data/inputData/"+self.name.getValueString().__str__() + "_step_" + self.step_id.__str__() + ".npy"
            print(filename)
            np.save(filename,dataArray)
        self.step_id += 1   