#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.animation import FuncAnimation
from dotenv import dotenv_values 
# importing movie py libraries
from moviepy.editor import VideoClip
from moviepy.video.io.bindings import mplfig_to_npimage



config = dotenv_values(".env")
# Filepaths for centerline data
filepath = config["currentDirectory"] +"data/stateData/" 
filenamePrefix = "stateExporter_step_" 
filenameSuffix = ".npy"

# Filepaths for input data
filepath_inputs = config["currentDirectory"] +"data/inputData/" 
filenamePrefix_inputs = "inputExporter_step_" 
filenameSuffix_inputs = ".npy"
# Filepaths for centerline data
filepath_centerlines = config["currentDirectory"] +"data/centerlineData/" 
filenamePrefix_centerlines = "centerlineExporter_step_" 
filenameSuffix_centerlines = ".npy"


# configure video to be saved
savePath = config["currentDirectory"] +"data/visualizations/" 
numFiles = 1000#np.size(files) 
print(numFiles)  





# Initialize array for holding state data
filename = filepath + filenamePrefix + "0" + filenameSuffix
# Read in data from file  
data = np.array([np.array(np.load(filename)).flatten()])

# Initialize array for holding input data
filename_inputs = filepath_inputs + filenamePrefix_inputs + "0" + filenameSuffix_inputs
inputData = np.array([np.array(np.load(filename_inputs)).flatten()])

# Initialize array for holding centerline data 
filename_centerlines = filepath_centerlines + filenamePrefix_centerlines+"0" + filenameSuffix_centerlines 
centerlineData = np.array([np.array(np.load(filename_centerlines)).flatten()])


# form matrices of data
for i in range(numFiles):
    print(i)
    filename = filepath + filenamePrefix + int(round(i)).__str__() + filenameSuffix
    filename_inputs = filepath_inputs + filenamePrefix_inputs + int(round(i)).__str__() + filenameSuffix_inputs
    filename_centerlines = filepath_centerlines + filenamePrefix_centerlines+int(round(i)).__str__() + filenameSuffix_centerlines 

    # Read in data from files
    xs = data[:,0]
    ys = data[:,1]
    zs = data[:,2]
 
    newStateData = np.array([np.array(np.load(filename)).flatten()]) 
    data = np.append(data,newStateData,axis=0)

    newInputData = np.array([np.array(np.load(filename_inputs)).flatten()]) 
    inputData = np.append(inputData,newInputData,axis=0)

    newCenterlineData = np.array([np.array(np.load(filename_centerlines)).flatten()]) 
    centerlineData = np.append(centerlineData,newCenterlineData,axis=0)
print("done")

outfileName = config["currentDirectory"] +"data/processedData/"+"cleanedArrays.npz"

np.savez(outfileName, stateData = data, inputData = inputData,centerlineData=centerlineData)


