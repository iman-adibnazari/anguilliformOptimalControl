#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from dotenv import dotenv_values 
# importing movie py libraries
from moviepy.editor import VideoClip
from moviepy.video.io.bindings import mplfig_to_npimage



config = dotenv_values(".env")
filepath = config["currentDirectory"] +"data/centerlineData/" 
filenamePrefix = "centerlineExporter_step_" 
filenameSuffix = ".npy"

savePath = config["currentDirectory"] +"data/visualizations/" 
files = dir_list = os.listdir(filepath)
numFiles = np.size(files) 
print(numFiles) 
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

duration = 5
fps = numFiles/duration

# function that draws each frame of the animation
def animate(t):
    i = t*fps
    filename = filepath + filenamePrefix + int(round(i)).__str__() + filenameSuffix
    # Read in data from file  
    data = np.array(np.load(filename))  
    print(i)
    xs = data[:,0]
    ys = data[:,1]
    zs = data[:,2]

    ax.clear()
    ax.scatter(xs, ys, zs, marker='o')
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_xlim(0,200)
    ax.set_zlim(-60,60)

    ax.view_init(0, -90, 0)

    # fig.savefig(savePath+i.__str__()+".png")
    # plt.show(block=True)
    # returning numpy image
    return mplfig_to_npimage(fig)


# # run the animation
# ani = FuncAnimation(fig, animate, frames=100, interval=100, repeat=False)

# plt.show()

# Save video
animation = VideoClip(animate, duration = duration)
animation.write_gif(savePath + "test.gif",fps=fps)

