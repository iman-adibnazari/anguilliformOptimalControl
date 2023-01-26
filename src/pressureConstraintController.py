#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key

''' 
deltaT - timestep
policy - function handle for the control policy p(x,t) where x is the full state vector and t is time
'''
class PressureConstraintController(Sofa.Core.Controller):
    def __init__(self,deltaT, policy, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.step_id = 0 
        self.deltaT = deltaT
        self.node = args[0] 
        self.policy=policy
        self.pressureConstraint = self.node.getObject('SurfacePressureConstraint')
        self.pressureConstraint.value = [0]

    def onAnimateBeginEvent(self, e):
        t=self.step_id*self.deltaT # get current time step
        
        self.pressureConstraint.value = [self.policy(0,t)]

        self.step_id += 1