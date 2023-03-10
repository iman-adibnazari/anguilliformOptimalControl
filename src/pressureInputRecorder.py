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
class pressureInputRecorder(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.step_id = 0 
        self.segments = kwargs.get("segments")

    def onAnimateEndEvent(self, e):

        # visit each segment 
        x = np.empty((1,2))
        for ind,segment in enumerate(self.segments): 

            with segment.chamber0.SurfacePressureConstraint.value.writeableArray() as c0:
                with segment.chamber1.SurfacePressureConstraint.value.writeableArray() as c1:
                    temp = np.array([[c0[0],c1[0]]])
                    if ind ==0:
                        x = temp
                    else: 
                        x = np.concatenate((x,temp))



        filename = config["currentDirectory"]+"data/inputData/"+self.name.getValueString().__str__() + "_step_" + self.step_id.__str__() + ".npy"
        np.save(filename,x)

        self.step_id += 1   