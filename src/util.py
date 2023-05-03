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

def cleanData(stateDataFilePathPrefix = "stateExporter_policySeed_0_step_", 
              inputDataFilePathPrefix = "inputExporter_policySeed_0_step_",
              centerlineDataFilePathPrefix = "centerlineExporter_policySeed_0_step_",
              numTimeSteps = 1000, # number of timesteps in each episode
              outFilename = "processedData_policySeed_0.npz"):


    # Filepaths for centerline data
    filepath = config["currentDirectory"] +"data/stateData/" 
    filenamePrefix = stateDataFilePathPrefix 
    filenameSuffix = ".npy"

    # Filepaths for input data
    filepath_inputs = config["currentDirectory"] +"data/inputData/" 
    filenamePrefix_inputs = inputDataFilePathPrefix
    filenameSuffix_inputs = ".npy"
    # Filepaths for centerline data
    filepath_centerlines = config["currentDirectory"] +"data/centerlineData/" 
    filenamePrefix_centerlines = centerlineDataFilePathPrefix
    filenameSuffix_centerlines = ".npy"


    # configure video to be saved
    savePath = config["currentDirectory"] +"data/visualizations/" 
    numFiles = numTimeSteps#np.size(files) 





    # Initialize array for holding state data
    filename = filepath + filenamePrefix + "0" + filenameSuffix
    # Read in data from file  
    data = np.array([np.array(np.load(filename)).flatten()])
    dataFull = np.zeros((numFiles,data.size))
    dataFull[0,:] = data
    # Initialize array for holding input data
    filename_inputs = filepath_inputs + filenamePrefix_inputs + "0" + filenameSuffix_inputs
    inputData = np.array([np.array(np.load(filename_inputs)).flatten()])
    inputDataFull = np.zeros((numFiles,inputData.size))
    inputDataFull[0,:] = inputData

    # Initialize array for holding centerline data 
    filename_centerlines = filepath_centerlines + filenamePrefix_centerlines+"0" + filenameSuffix_centerlines 
    centerlineData = np.array([np.array(np.load(filename_centerlines)).flatten()])
    centerlineDataFull = np.zeros((numFiles,centerlineData.size))
    centerlineDataFull[0,:] = centerlineData


    # form matrices of data
    for i in range(1,numFiles):
        filename = filepath + filenamePrefix + int(round(i)).__str__() + filenameSuffix
        filename_inputs = filepath_inputs + filenamePrefix_inputs + int(round(i)).__str__() + filenameSuffix_inputs
        filename_centerlines = filepath_centerlines + filenamePrefix_centerlines+int(round(i)).__str__() + filenameSuffix_centerlines 

        # Read in data from files
        xs = data[:,0]
        ys = data[:,1]
        zs = data[:,2]
    
        newStateData = np.array([np.array(np.load(filename)).flatten()]) 
        # data = np.append(data,newStateData,axis=0)

        newInputData = np.array([np.array(np.load(filename_inputs)).flatten()]) 
        # inputData = np.append(inputData,newInputData,axis=0)

        newCenterlineData = np.array([np.array(np.load(filename_centerlines)).flatten()]) 
        # centerlineData = np.append(centerlineData,newCenterlineData,axis=0)


        dataFull[i,:] = newStateData
        inputDataFull[i,:] = newInputData
        centerlineDataFull[i,:] = newCenterlineData

    print("done")

    outfileName = config["currentDirectory"] +"data/processedData/"+outFilename

    np.savez(outfileName, stateData = dataFull, inputData = inputDataFull,centerlineData=centerlineDataFull)

def cleanDataMultiEpisodes(numEpisodes = 1):
    for i_episode in range(numEpisodes):
        stateDataFilePathPrefix = f"stateExporter_policySeed_{i_episode}_step_" 
        inputDataFilePathPrefix = f"inputExporter_policySeed_{i_episode.__str__()}_step_"
        centerlineDataFilePathPrefix = f"centerlineExporter_policySeed_{i_episode.__str__()}_step_"
        numTimeSteps = 1000 # number of timesteps in each episode
        outFilename = f"processedData_policySeed_{i_episode}.npz"
        cleanData(stateDataFilePathPrefix = stateDataFilePathPrefix,
                  inputDataFilePathPrefix = inputDataFilePathPrefix,
                  centerlineDataFilePathPrefix = centerlineDataFilePathPrefix,
                  numTimeSteps = numTimeSteps,
                  outFilename = outFilename)
        


if __name__ == '__main__':
    cleanDataMultiEpisodes(numEpisodes=50)