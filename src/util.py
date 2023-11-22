#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import os
import glob
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.animation import FuncAnimation
from dotenv import dotenv_values 
# importing movie py libraries
from moviepy.editor import VideoClip
from moviepy.video.io.bindings import mplfig_to_npimage
from scipy.io import savemat
import hdf5storage
import h5py


config = dotenv_values(".env")
def reduceCenterline(n, points, N_local = 20, 
                     permuteCenterlineReduction = False, # If true, that means centerline should be discretized along the z axis instead of the x axis as usual
):
    '''
    reduces the number of points in the centerline data to n discretized points
        Parameters:
            n: number of discretized points to reduce to
            points: centerline data
            N_local: number of points to average over closest to the discretized points
    
        Returns:
            reducedPoints: reduced centerline data with n discretized points projected into the x-z plane
    '''
    # initialize reducedPoints array
    reducedPoints = np.zeros((n,2))
    # get number of points in original centerline data
    numPoints = points.shape[0]
    if permuteCenterlineReduction:
        # swap x and z coordinates
        points = points[:,[2,1,0]]
    # get limits of x coordinates
    xMin = np.min(points[:,0])
    xMax = np.max(points[:,0])
    # get distance between reduced points
    dx = (xMax-xMin)/(n-1)
    # average z value of points within each x coordinate range
    for i in range(n):
        x = xMin + i*dx
        # get indices of N_local points closest to discretized point
        indices = np.argsort(np.abs(points[:,0]-x))[:N_local]
        # indices = np.where(np.logical_and(points[:,0] >= x-dx/2.0, points[:,0] < x+dx/2.0))
        # get average z value of points within x coordinate range
        z = np.mean(points[indices,2])
        # set reduced point
        reducedPoints[i,:] = np.array([x,z])

    if permuteCenterlineReduction:
        # swap x and z coordinates back
        reducedPoints = reducedPoints[:,[1,0]]
    return reducedPoints


def reduceCenterlineFullEpisode(n, centerlineData, N_local = 20,
                                permuteCenterlineReduction = False, saveAvgMat = True# If true, that means centerline should be discretized along the z axis instead of the x axis as usual
                                ):
    '''
    reduces the number of points in the centerline data to n discretized points to form full reduced output matrix for the episode
        Parameters:
            n: number of discretized points to reduce to
            centerlineData: centerline data. Rows correspond to timesteps and columns correspond to centerline states 
            N_local: number of points to average over closest to the discretized points
    
        Returns:
            reducedPoints: reduced centerline data with n discretized points projected into the x-z plane. Rows correspond to state data rows and columns correspond to timesteps
    '''
    # Parameters for reduced centerline
    n_redCenterline = n # number of discretized points in reduced centerline
    n_redLocal = N_local # number of points to average over closest to the discretized points
    discretizeAlongZ = permuteCenterlineReduction # discretize along z axis or x axis
    points = centerlineData # centerline data


    #### Form selection matrix for centerline reduction ####
    # Grab points for first timestep and reshape
    centerline_t = np.reshape(points[0,:],(-1,3))
    # If discretizing along z axis, swap x and z coordinates
    if discretizeAlongZ:
        centerline_t = np.flip(centerline_t,1)

    # Find bounds for centerline data
    xmin = np.min(centerline_t[:,0])
    xmax = np.max(centerline_t[:,0])
    # compute average z value of centerline
    zavg = 0#np.mean(centerline_t[:,2])
    # find distance between discretization points
    dx = (xmax-xmin)/(n_redCenterline-1)
    # initialize averaging matrix mapping from full centerline to reduced centerline
    avgMatrix = np.zeros((n_redCenterline,centerline_t.shape[0]))
    # loop over discretized points to find closest points in full centerline
    for i in range(n_redCenterline):
        # find n_redLocal closest points in full centerline closest to the discretized point in the x direction and the average z value
        idx = np.argsort((centerline_t[:,0]-xmin-i*dx)**2+np.abs(centerline_t[:,2]-zavg)**2)[0:n_redLocal]
        # idx = np.argsort(np.abs(centerline_t[:,0]-(xmin+i*dx)))[:n_redLocal]
        # populate averaging matrix
        avgMatrix[i,idx] = 1/n_redLocal

    #### Compute reduced centerline ####
    # form snapshot matrix of x coordinates of reduced centerline
    xMat = np.zeros((centerline_t.shape[0],points.shape[0]))
    for i in range(points.shape[0]):
        centerline_t = np.reshape(points[i,:],(-1,3))
        if discretizeAlongZ:
            centerline_t = np.flip(centerline_t,1)
        xMat[:,i] = centerline_t[:,0]
    # form snapshot matrix of z coordinates of full centerline
    zMat = np.zeros((centerline_t.shape[0],points.shape[0]))
    for i in range(points.shape[0]):
        centerline_t = np.reshape(points[i,:],(-1,3))
        if discretizeAlongZ:
            centerline_t = np.flip(centerline_t,1)
        zMat[:,i] = centerline_t[:,2]


    # compute reduced centerline coordinates
    xRed = np.matmul(avgMatrix,xMat)
    zRed = np.matmul(avgMatrix,zMat)

    # form reduced centerline matrix for output
    reducedPoints = np.zeros((points.shape[0],2*n_redCenterline))
    for i in range(points.shape[0]):
        # form matrix that iterates between x and z coordinates
        redCenterline_t = np.concatenate((xRed[:,i].reshape(-1,1),zRed[:,i].reshape(-1,1)),axis=1)
        # if discretizing along z axis, swap x and z coordinates
        if discretizeAlongZ:
            redCenterline_t = np.flip(redCenterline_t,1)
        # populate reduced centerline matrix
        reducedPoints[i,:] = redCenterline_t.flatten()
    if saveAvgMat:
        np.save(config["currentDirectory"] +"avgMatrix.npy",avgMatrix)

    return reducedPoints



def cleanData(stateDataFilePathPrefix = "stateExporter_policySeed_0_step_", 
              inputDataFilePathPrefix = "inputExporter_policySeed_0_step_",
              centerlineDataFilePathPrefix = "centerlineExporter_policySeed_0_step_",
              numTimeSteps = 1000, # number of timesteps in each episode
              outFilename = "processedData_policySeed_0.npz",
              permuteCenterlineReduction = False, # If true, that means centerline should be discretized along the z axis instead of the x axis as usual
              n_redCenterline = 20, # number of discretized points in reduced centerline
              n_redLocal = 20, # number of points to average over closest to the discretized points
              ):
    
    # Parameters for reduced centerline
    # n_redCenterline = 30 # number of discretized points in reduced centerline
    # n_redLocal = 20 # number of points to average over closest to the discretized points



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

    # # Initialize array for holding reduced centerline data
    # reducedCenterlineDataFull = np.zeros((numFiles,2*n_redCenterline))
    # reducedCenterlineDataFull[0,:] = reduceCenterline(n_redCenterline, centerlineData.reshape((-1,3)), n_redLocal).flatten()

        
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
    # generate reduced centerline data
    reducedCenterlineDataFull = reduceCenterlineFullEpisode(n_redCenterline, centerlineDataFull, n_redLocal, permuteCenterlineReduction = permuteCenterlineReduction)


    print("done")

    outfileName = config["currentDirectory"] +"data/processedData/"+outFilename

    np.savez(outfileName, stateData = dataFull, inputData = inputDataFull,centerlineData=centerlineDataFull,reducedCenterlineData = reducedCenterlineDataFull)

def cleanDataMultiEpisodes(numEpisodes = 1,numTimeSteps = 200,permuteCenterlineReduction = False):
    for i_episode in range(numEpisodes):
        stateDataFilePathPrefix = f"stateExporter_policySeed_{i_episode}_step_" 
        inputDataFilePathPrefix = f"inputExporter_policySeed_{i_episode.__str__()}_step_"
        centerlineDataFilePathPrefix = f"centerlineExporter_policySeed_{i_episode.__str__()}_step_"
        outFilename = f"processedData_policySeed_{i_episode}.npz"
        cleanData(stateDataFilePathPrefix = stateDataFilePathPrefix,
                  inputDataFilePathPrefix = inputDataFilePathPrefix,
                  centerlineDataFilePathPrefix = centerlineDataFilePathPrefix,
                  numTimeSteps = numTimeSteps,
                  outFilename = outFilename,
                  permuteCenterlineReduction=permuteCenterlineReduction)


def processedNpzToMatlab():
    filepath = config["currentDirectory"] + "data/processedData/processedData_policySeed_0.npz"

    npzFiles = glob.glob(config["currentDirectory"] + "data/processedData/"+"*.npz")

    for f in npzFiles:
        fm = os.path.splitext(f)[0]+'.mat'
        d = np.load(f)
        savemat(fm, d)
        print('generated ', fm, 'from', f)

def generateDataSetFromProcessedNPZs(saveMatlab = False, savehdf5 = False):
    npzFiles = glob.glob(config["currentDirectory"] + "data/processedData/"+"*.npz")
    numFiles = len(npzFiles)
    # Read in first file to get size of data
    d = np.load(npzFiles[0])
    stateData = d['stateData'].transpose()
    inputData = d['inputData'].transpose()
    centerlineData = d['centerlineData'].transpose()
    reducedCenterlineData = d['reducedCenterlineData'].transpose()
    numTimeSteps = stateData.shape[1]
    numStates = stateData.shape[0]
    numInputs = inputData.shape[0]
    dimCenterline = centerlineData.shape[0]
    dimReducedCenterline = reducedCenterlineData.shape[0]
    # Initialize arrays for holding data
    stateDataFull = np.zeros((numStates,numTimeSteps,numFiles))
    inputDataFull = np.zeros((numInputs,numTimeSteps,numFiles))
    centerlineDataFull = np.zeros((dimCenterline,numTimeSteps,numFiles))
    reducedCenterlineDataFull = np.zeros((dimReducedCenterline,numTimeSteps,numFiles))
    # Read in data from files
    for i in range(numFiles):
        d = np.load(npzFiles[i])
        stateDataFull[:,:,i] = d['stateData'].transpose()
        inputDataFull[:,:,i] = d['inputData'].transpose()
        centerlineDataFull[:,:,i] = d['centerlineData'].transpose()
        reducedCenterlineDataFull[:,:,i] = d['reducedCenterlineData'].transpose()
    # Save data to npz file
    outfileName = config["currentDirectory"] +"data/processedData/processedDataSet.npz"
    np.savez(outfileName, stateData = stateDataFull, inputData = inputDataFull,centerlineData=centerlineDataFull,reducedCenterlineData = reducedCenterlineDataFull)
    if saveMatlab:
        outfileNameMat = config["currentDirectory"] +"data/processedData/processedDataSet.mat"
        # savemat(outfileNameMat,{'stateData':stateDataFull,'inputData':inputDataFull,'centerlineData':centerlineDataFull,'reducedCenterlineData':reducedCenterlineDataFull})
        hdf5storage.savemat(outfileNameMat,{'stateData':stateDataFull,'inputData':inputDataFull,'centerlineData':centerlineDataFull,'reducedCenterlineData':reducedCenterlineDataFull},format = '7.3', matlab_compatible=True,compress=False)
    if savehdf5:
        outfileNamehdf5 = config["currentDirectory"] +"data/processedData/processedDataSet.hdf5"
        with h5py.File(outfileNamehdf5, 'w') as f:
            f.create_dataset('stateData', data=stateDataFull)
            f.create_dataset('inputData', data=inputDataFull)
            f.create_dataset('centerlineData', data=centerlineDataFull)
            f.create_dataset('reducedCenterlineData', data=reducedCenterlineDataFull)
def animateReducedCenterline( dataFile = config["currentDirectory"] +"data/processedData/processedData_policySeed_0.npz",
savePath = config["currentDirectory"] +"data/visualizations/",
saveName = "reducedCenterline.mp4",
xlim_max = 10,
xlim_min = -1150, 
zlim_max = 200,
zlim_min = -200,
numTimeSteps = 1000
):
    numFrames = numTimeSteps
    fps = 30
    duration = numFrames/fps
    # Set up figure for animation
    fig, ax = plt.subplots()
    # Read in data
    d = np.load(dataFile)
    centerlineData_reduced = d['reducedCenterlineData'].transpose()
    # Animation callback
    def animate(t):
        # global centerlineData
        # global centerlineData_reduced
        # Get frame index
        i = int(round(t*fps))
        # # Compute reduced centerline
        # centerlineData_current = centerlineData[i,:]
        # centerlineData_current = np.reshape(centerlineData_current,(int(centerlineData_current.size/3),3))
        # centerlineData_reduced = reduceCenterline(n,centerlineData_current,N_local)
        # Plot mesh points from original centerline
        ax.clear()
        # ax.scatter(centerlineData_current[:,0],centerlineData_current[:,2],marker='.',color='blue')
        # Reshape reduced centerline for plotting of current frame
        centerlineData_reduced_current = centerlineData_reduced[:,i] 
        
        centerlineData_reduced_current = np.reshape(centerlineData_reduced_current,(int(centerlineData_reduced_current.size/2),2))
        
        # Plot reduced centerline
        ax.plot(centerlineData_reduced_current[:,0],centerlineData_reduced_current[:,1],color='red')
        ax.set_xlim(xlim_min,xlim_max)
        ax.set_ylim(zlim_min,zlim_max)
        # ax.set_aspect('equal')
        ax.set_title("Reduced Centerline")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("z (m)")
        return mplfig_to_npimage(fig)

    # Create animation
    animation = VideoClip(animate, duration=duration)
    animation.write_videofile(savePath+saveName, fps=fps)

def mat2hdf5(filepath = config["currentDirectory"] +"data/archivedDataSets/FullAssembly_Constrained_FullSetForICRA/lopinf_rom_11_training.mat", outfilePath = config["currentDirectory"] +"data/archivedDataSets/FullAssembly_Constrained_FullSetForICRA/lopinf_rom_11_training.hdf5"):
    # Converts .mat file to .hdf5 file
    # Read in large .mat file
    data = hdf5storage.loadmat(filepath)
    # Get keys for data
    keys = data.keys()
    # save data to hdf5 file
    with h5py.File(outfilePath, 'w') as f:
        for key in keys:
            print(key)
            f.create_dataset(key, data=data[key],maxshape=(None,None,None),chunks=(4096,4096,1))

def combineDataSets(inFile1 = config["currentDirectory"] +"data/archivedDataSets/FullAssembly_Constrained_FullSetForICRA/trainingSet_test.hdf5", inFile2 = config["currentDirectory"] +"data/archivedDataSets/FullAssembly_Constrained_FullSetForICRA/testSet.hdf5", outFile = config["currentDirectory"] +"data/archivedDataSets/FullAssembly_Constrained_FullSetForICRA/fullDataSet.hdf5"):
    # Read in hdf5s for each dataset
    with h5py.File(inFile1, 'a') as ff4:
        with h5py.File(inFile2, 'r') as ff3:
            print("got here 0")
            # Move stateData
            state_dim1 = ff4['stateData'].shape[0]
            state_dim2 = ff4['stateData'].shape[1]
            state_dim3 = ff4['stateData'].shape[2]
            state_newDim3 = ff3['stateData'].shape[2] + state_dim3
            ff4['stateData'].resize((state_dim1,state_dim2,state_newDim3)) 
            for i in range(state_newDim3-state_dim3):
                ff4['stateData'][:,:,state_dim3+i] = ff3['stateData'][:,:,i]
                ff4.flush()
            print("got here 1")
            # Move inputData
            input_dim1 = ff4['inputData'].shape[0]
            input_dim2 = ff4['inputData'].shape[1]
            input_dim3 = ff4['inputData'].shape[2]
            input_newDim3 = ff3['inputData'].shape[2] + input_dim3
            ff4['inputData'].resize((input_dim1,input_dim2,input_newDim3))
            for i in range(input_newDim3-input_dim3):
                ff4['inputData'][:,:,input_dim3+i] = ff3['inputData'][:,:,i]
                ff4.flush()
            print("got here 2")
            # Move centerlineData
            centerline_dim1 = ff4['centerlineData'].shape[0]
            centerline_dim2 = ff4['centerlineData'].shape[1]
            centerline_dim3 = ff4['centerlineData'].shape[2]
            centerline_newDim3 = ff3['centerlineData'].shape[2] + centerline_dim3
            ff4['centerlineData'].resize((centerline_dim1,centerline_dim2,centerline_newDim3))
            for i in range(centerline_newDim3-centerline_dim3):
                ff4['centerlineData'][:,:,centerline_dim3+i] = ff3['centerlineData'][:,:,i]
                ff4.flush()
            print("got here 3")
            # Move reducedCenterlineData
            reducedCenterline_dim1 = ff4['reducedCenterlineData'].shape[0]
            reducedCenterline_dim2 = ff4['reducedCenterlineData'].shape[1]
            reducedCenterline_dim3 = ff4['reducedCenterlineData'].shape[2]
            reducedCenterline_newDim3 = ff3['reducedCenterlineData'].shape[2] + reducedCenterline_dim3
            ff4['reducedCenterlineData'].resize((reducedCenterline_dim1,reducedCenterline_dim2,reducedCenterline_newDim3))
            for i in range(reducedCenterline_newDim3-reducedCenterline_dim3):
                ff4['reducedCenterlineData'][:,:,reducedCenterline_dim3+i] = ff3['reducedCenterlineData'][:,:,i]
                ff4.flush()
            
         




    #     stateData1 = f['stateData'][:,:,:]
    #     inputData1 = f['inputData'][:,:,:]
    #     centerlineData1 = f['centerlineData'][:,:,:]
    #     reducedCenterlineData1 = f['reducedCenterlineData'][:,:,:]
    # with h5py.File(inFile2, 'r') as f:
    #     stateData2 = f['stateData'][:,:,:]
    #     inputData2 = f['inputData'][:,:,:]
    #     centerlineData2 = f['centerlineData'][:,:,:]
    #     reducedCenterlineData2 = f['reducedCenterlineData'][:,:,:]
    # # Write concatenated data to new hdf5
    # with h5py.File(outFile, 'w') as f:
    #     f.create_dataset('stateData', data=np.concatenate((stateData1,stateData2),axis=2))
    #     f.create_dataset('inputData', data=np.concatenate((inputData1,inputData2),axis=2))
    #     f.create_dataset('centerlineData', data=np.concatenate((centerlineData1,centerlineData2),axis=2))
    #     f.create_dataset('reducedCenterlineData', data=np.concatenate((reducedCenterlineData1,reducedCenterlineData2),axis=2))
    

if __name__ == '__main__':
    # cleanDataMultiEpisodes(numEpisodes=50)
    # cleanData(numTimeSteps=3000, outFilename="processedData_policySeed_0.npz",permuteCenterlineReduction=False)
    
    cleanDataMultiEpisodes(numEpisodes=1,numTimeSteps=1750,permuteCenterlineReduction=False, )
    # generateDataSetFromProcessedNPZs(saveMatlab=False, savehdf5 = True)
    # animateReducedCenterline(numTimeSteps=1999,saveName="reducedCenterline_fullAssembly_constrained.mp4")


    # mat2hdf5(filepath=config["currentDirectory"] +"data/archivedDataSets/FullAssembly_Constrained_FullSetForICRA/trainingSet.mat", outfilePath=config["currentDirectory"] +"data/archivedDataSets/FullAssembly_Constrained_FullSetForICRA/trainingSet_test.hdf5")

    # combineDataSets()