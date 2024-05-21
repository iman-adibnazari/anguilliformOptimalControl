
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
import numpy as np
from dotenv import dotenv_values 

config = dotenv_values(".env")
# import meshio

# Setup controller to save and export data

class centerlineStateExporter (Sofa.Core.Controller):
    def __init__(self, filetype,*args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.step_id = 0

        self.fileType = filetype #0 corresponds to saving everthing as numpy files, 1 corresponds to saving everything as vtks
        

    def onAnimateBeginEvent(self, e):
        a= self.getContext().centerline_roi
        b= self.getContext()
        with self.getContext().centerline_roi.indices.writeableArray() as indices:

            with self.getContext().tetras.position.writeableArray() as wa:
                x = wa[indices,:]
        # x  = self.getContext().centerline_roi.pointsInROI.value
        # print("indices are: "+ x.__str__())

        # # x0 = self.getContext().tetras.rest_position.array()
        # # u  = x - x0
        # # print(u)
        
        # # cells = self.getContext().topology.tetrahedra.array()
        # # von_mises = self.getContext().ff.vonMisesPerNode.array()
        filename = config["currentDirectory"]+"data/centerlineData/"+self.name.getValueString().__str__() + "_step_" + self.step_id.__str__() + ".npy"
        np.save(filename,x)
        # print(f'Mesh exported at {filename}')
        self.step_id += 1

