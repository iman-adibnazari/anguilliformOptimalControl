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
filepath = config["currentDirectory"] +"data/centerlineData/" 
filenamePrefix = "centerlineExporter_policySeed_1_step_" 
filenameSuffix = ".npy"

# Filepaths for input data
filepath_inputs = config["currentDirectory"] +"data/inputData/" 
filenamePrefix_inputs = "inputExporter_policySeed_1_step_" 
filenameSuffix_inputs = ".npy"


# configure video to be saved
savePath = config["currentDirectory"] +"data/visualizations/" 
# files = dir_list = os.listdir(filepath)
numFiles = 200#np.size(files) 
print(numFiles)  
# Setup figure for plotting data
fig = plt.figure()
gs = mpl.gridspec.GridSpec(2,2,wspace=0.25,hspace=0.25)
# ax = fig.add_subplot(gs[0,:],projection='3d')
ax = fig.add_subplot(gs[0,:])
ax1=fig.add_subplot(gs[1,0])
ax2=fig.add_subplot(gs[1,1])

# duration = 5
# fps = numFiles/duration
fps = 40
duration = numFiles/fps

# find limits for plotting 
filename = filepath + filenamePrefix + "0" + filenameSuffix
# Read in data from file  
data = np.array(np.load(filename))  
xs = data[:,0]
ys = data[:,1]
zs = data[:,2] 

xlim_max = np.max(xs)
xlim_min = np.min(xs)
ylim_max = np.max(ys)
ylim_min = np.min(ys)
zlim_max = np.max(zs)
zlim_min = np.min(zs)

xlim_max = xlim_max + (xlim_max-xlim_min)*0.1 
xlim_min = xlim_min - (xlim_max-xlim_min)*0.1 
ylim_max = ylim_max + (ylim_max-ylim_min)*0.1 
ylim_min = ylim_min - (ylim_max-ylim_min)*0.1 
zlim_max = zlim_max + (zlim_max-zlim_min)*2
zlim_min = zlim_min - (zlim_max-zlim_min)*2 


ulim_max = 0.2
ulim_min = -0.2

# Initialize arrays for holding input data
filename_inputs = filepath_inputs + filenamePrefix_inputs + "0" + filenameSuffix_inputs


inputData = np.array([np.array(np.load(filename_inputs)).flatten()])
# function that draws each frame of the animation
def animate(t):
    global inputData
    i = t*fps
    filename = filepath + filenamePrefix + int(round(i)).__str__() + filenameSuffix
    filename_inputs = filepath_inputs + filenamePrefix_inputs + int(round(i)).__str__() + filenameSuffix_inputs

    # Read in data from files
    data = np.array(np.load(filename))  
    print(i)
    xs = data[:,0]
    ys = data[:,1]
    zs = data[:,2]

    newInputData = np.array([np.array(np.load(filename_inputs)).flatten()]) 
    inputData = np.append(inputData,newInputData,axis=0)

    # plot centerline data 
    ax.clear()
    ax.scatter(xs, zs, marker='o')
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("z [mm]")
    # ax.set_ylabel("y [mm]")
    # ax.set_zlabel("z [mm]")
    ax.set_xlim(xlim_min,xlim_max)
    # ax.set_zlim(zlim_min,zlim_max)
    ax.set_ylim(zlim_min,zlim_max)

    # ax.view_init(0, -90, 0)


    # Plot segment 0 pressures 
    ax1.clear()
    ax1.plot(inputData[0:-1,0], inputData[0:-1,1],color = 'blue')
    ax1.scatter(inputData[-1,0], inputData[-1,1], marker='o',color = 'red')
    ax1.set_xlim(ulim_min,ulim_max)
    # ax.set_zlim(zlim_min,zlim_max)
    ax1.set_ylim(ulim_min,ulim_max)

    # Plot segment 1 pressures
    ax2.clear()
    ax2.plot(inputData[0:-1,2], inputData[0:-1,3],color = 'blue')
    ax2.scatter(inputData[-1,2], inputData[-1,3], marker='o',color = 'red')
    ax2.set_xlim(ulim_min,ulim_max)
    # ax.set_zlim(zlim_min,zlim_max)
    ax2.set_ylim(ulim_min,ulim_max)

    # fig.savefig(savePath+i.__str__()+".png")
    # plt.show(block=True)
    # returning numpy image
    return mplfig_to_npimage(fig)


# # run the animation
# ani = FuncAnimation(fig, animate, frames=100, interval=100, repeat=False)

# plt.show()

# Save video
animation = VideoClip(animate, duration = duration)
animation.write_gif(savePath + "test4.gif",fps=fps)

