from scipy.io import savemat
import numpy as np
import glob
import os
from dotenv import dotenv_values 
config = dotenv_values(".env")

filepath = config["currentDirectory"] + "data/processedData/processedData_policySeed_0.npz"

npzFiles = glob.glob(config["currentDirectory"] + "data/processedData/"+"*.npz")
for f in npzFiles:
    fm = os.path.splitext(f)[0]+'.mat'
    d = np.load(f)
    savemat(fm, d)
    print('generated ', fm, 'from', f)
