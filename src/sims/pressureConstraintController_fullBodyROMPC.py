#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key
import numpy as np
from dotenv import dotenv_values 
from rhcPolicy_ERA import rhcPolicy_ERA
from rhcPolicy_LOpInf import rhcPolicy_LOpInf
from rhcPolicy_DMDc import rhcPolicy_DMDc
# from rhcPolicy_ERA import rhcPolicy_ERA
from rhcPolicy_randomizedOutput import rhcPolicy_randomizedOutput
from stateEstimator_ERA import stateEstimator_ERA
from stateEstimator_LOpInf import stateEstimator_LOpInf
from stateEstimator_DMDc import stateEstimator_DMDc
from centralizedPolicy_sinusoid import centralizedPolicy_sinusoid
import logging
import h5py
import psycopg2
from psycopg2.extras import execute_values
import pickle
import time

config = dotenv_values(".env")
''' 
deltaT - timestep
policy - function handle for the control policy p(x,t) where x is the full state vector and t is time
saveOutput - 0 to not save the outputs and 1 to save the outputs in npy files
'''
def get_db_connection():
    conn = psycopg2.connect(
        dbname='simDB',
        user='user',
        password='password',
        host='localhost',
        port='5432'
    )
    return conn


def insert_simulation_data(trial_id, timestep, simulation_time,x_hat,y_ref, input_data, output_data, state_data):
    conn = get_db_connection()
    cur = conn.cursor()
    
    # Serialize NumPy arrays
    input_data_bin = pickle.dumps(input_data)
    output_data_bin = pickle.dumps(output_data)
    state_data_bin = pickle.dumps(state_data)
    x_hat_bin = pickle.dumps(x_hat)
    y_ref_bin = pickle.dumps(y_ref)
    
    # Insert data into the database
    cur.execute(
        '''
        INSERT INTO simulation_data 
        (trial_id, timestep, simulation_time, x_hat, y_ref, input_data, output_data, state_data) 
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
        ''',
        (trial_id, timestep, simulation_time,psycopg2.Binary(x_hat_bin), psycopg2.Binary(y_ref_bin), psycopg2.Binary(input_data_bin), psycopg2.Binary(output_data_bin), psycopg2.Binary(state_data_bin))
    )
    print("Inserted timestep "+timestep.__str__() + " into database")
    conn.commit()
    cur.close()
    conn.close()

class PressureConstraintController_fullBodyROMPC(Sofa.Core.Controller):
    def __init__(self,deltaT,chambers,segments, saveOutput,expParams, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.step_id = 0 
        self.length = 1114.1947932504659 # mm
        self.T = 1000 # prediction horizon
        self.deltaT = deltaT
        self.chambers = chambers
        self.saveOutput=saveOutput
        self.segments = segments
        self.modelName = expParams["modelName"]
        self.ref_a_max = expParams["ref_a_max"]
        self.ref_omega = expParams["ref_omega"]
        self.ref_k = expParams["ref_k"]
        self.rhcPolicy =  rhcPolicy_LOpInf(deltaT,T=self.T, systemMatFile = config["currentDirectory"]+f"data/archivedDataSets/ContiguousAssembly/ROMsWithObserverGains/{self.modelName}.mat") #centralizedPolicy_sinusoid(deltaT,T=self.T, systemMatFile = config["currentDirectory"]+"data/archivedDataSets/FullAssembly_Constrained_FullSetForRAL_goodMatParams/ROMsWithObserverGains/dmdcSystemMatricesAndGains_4dim_3train.mat") #rhcPolicy_randomizedOutput(deltaT,T=self.T, systemMatFile = config["currentDirectory"]+"data/archivedDataSets/FullAssembly_Constrained_FullSetForICRA/romSystemMatricesAndGains_22dim_3train_2test.mat") #rhcPolicy_ERA(deltaT,T=self.T, systemMatFile = config["currentDirectory"]+"data/archivedDataSets/FullAssembly_Constrained_FullSetForICRA/romSystemMatricesAndGains_22dim_3train_2test.mat")
        self.stateEstimator = stateEstimator_LOpInf(deltaT, logResults = True, systemMatFile = config["currentDirectory"]+f"data/archivedDataSets/ContiguousAssembly/ROMsWithObserverGains/{self.modelName}.mat")
        self.pressureConstraints = [chamber.getObject('SurfacePressureConstraint') for chamber in self.chambers]
        self.conn = get_db_connection()
        self.trial_id = expParams["trial_id"]
        # self.expParams = expParams
        for pressureConstraint in self.pressureConstraints:
            pressureConstraint.value = [0]
        # self.pressureConstraint = self.node.getObject('SurfacePressureConstraint')
        # self.pressureConstraint.value = [0] 
        
        # parameters for centerline reduction
        self.n_redCenterline = 20 # number of discretized points in reduced centerline
        self.n_redLocal = 20 # number of points to average over closest to the discretized points
        self.avgMatrix = np.zeros((self.n_redCenterline,self.n_redLocal))
        self.redCenterlineOffset = np.zeros((self.n_redCenterline*2)) 
        print(self.name.getValueString().__str__())
        self.saveVerificationSet = True

    # helper function to define reference trajectories
    # Function to provide coordinates of discretized centerline at a given time
    def generateReferenceCoords(self, time,numPoints=20,a_max=10,l=1114.1947932504659,k=4,omega=3,x_shift = 0,z_shift = 0):
        # Generate 10 times number of x coordinates as desired points
        x = np.linspace(0,l,numPoints*1000)
        dx = x[1]-x[0]
        # Compute integrand of arc length integral
        integrand = np.sqrt(1+(a_max/l*(np.exp(x/l-1)*(k*np.cos(k/l*x+omega*time)-np.sin(-k/l*x+omega*time))))**2)
        # Use first order quadrature to compute integral
        integrand = integrand*dx
        integrand = np.cumsum(integrand)
        # Find where the integral is closest to the desired arc length
        larc_des = np.linspace(0,l,numPoints)
        x_ref_0 = np.linspace(0,l,numPoints) # Initial x coordinate of each reference point
        x_ref = np.zeros(numPoints)
        z_ref = np.zeros(numPoints)
        for i in range(numPoints):
            idx = np.argmin(np.abs(integrand-larc_des[i]))
            x_ref[i] = x[idx]-x_ref_0[i] # We subtract off the initial x coordinate to make the first point at the origin
            # Compute z value at this point
            z_ref[i] = a_max*np.exp(x[idx]/l-1)*np.sin(k*x[idx]/l-omega*time)
        # z = z-z[0]
        x_ref = x_ref+x_shift
        z_ref = z_ref+z_shift
        # Flip sign of x coordinates to take into account coordinate frame facing out from the front of the eel
        x_ref = -x_ref
        # Flip order of reference coords
        x_ref = np.flip(x_ref)
        z_ref = np.flip(z_ref)
        y_ref = np.hstack((x_ref.reshape(-1,1),z_ref.reshape(-1,1))).reshape(-1,1).squeeze()
        return y_ref
    
    # Helper function to define reference trajectories using reference coords function above
    def generateReferenceTrajectory(self,time,numPoints=20,a_max=10,l=1114.1947932504659,k=4,omega=7,x_shift = 0,z_shift = 0, T=1, dt = 0.01, *args, **kwargs):
        '''
        T - time horizon of reference trajectory
        '''
        # Initialize reference trajectory
        y_ref = np.zeros((self.n_redCenterline*2,T))
        # Loop over time horizon
        for i in range(T):
            # Generate reference coordinates
            y_ref[:,i] = self.generateReferenceCoords(time = time+i*dt,numPoints=numPoints,a_max=a_max,l=l,k=k,omega=omega,x_shift=x_shift,z_shift=z_shift).reshape(-1,1).squeeze()
        return y_ref

    def onAnimateBeginEvent(self, e):
        t=self.step_id*self.deltaT # get current time step
        pressures = np.zeros(len(self.pressureConstraints))

        # get full centerline vector
        centerlinePts_full = np.empty((1,3))
        fullState = np.empty((1,3))
        for ind,segment in enumerate(self.segments): 

            with segment.state.position.writeableArray() as wa:
                if ind ==0:
                    fullState = wa 
                else: 
                    fullState = np.concatenate((fullState,wa))


            with segment.centerline_roi.indices.writeableArray() as indices:

                with segment.state.position.writeableArray() as wa:
                    temp = wa[indices,:]
                    if ind ==0:
                        centerlinePts_full = temp 
                    else: 
                        centerlinePts_full = np.concatenate((centerlinePts_full,temp))
        # Save centerline data for debugging
        # filename = config["currentDirectory"]+"data/centerlineData/"+"debug" + "_step_" + self.step_id.__str__() + ".npy"
        # np.save(filename,centerlinePts_full)
        ######### compute reduced output vector #########
        points = centerlinePts_full # centerline data
        centerline_t = points # make sure centerline is in the right shape

        #### On the first timestep form selection matrix and offset vector for centerline reduction ####
        if self.step_id == 0:
            # Grab points for first timestep and reshape
            # Find bounds for centerline data
            xmin = np.min(centerline_t[:,0])
            xmax = np.max(centerline_t[:,0])
            # compute average z value of centerline
            zavg = 0 #np.mean(centerline_t[:,2])
            # find distance between discretization points
            dx = (xmax-xmin)/(self.n_redCenterline-1)
            # initialize averaging matrix mapping from full centerline to reduced centerline
            self.avgMatrix = np.zeros((self.n_redCenterline,centerline_t.shape[0]))
            # loop over discretized points to find closest points in full centerline
            for i in range(self.n_redCenterline):
                # find n_redLocal closest points in full centerline closest to the discretized point in the x direction and the average z value
                idx = np.argsort((centerline_t[:,0]-xmin-i*dx)**2+np.abs(centerline_t[:,2]-zavg)**2)[0:self.n_redLocal]
    
                # populate averaging matrix
                self.avgMatrix[i,idx] = 1/self.n_redLocal

        #### Compute reduced centerline ####
        # Grab all x points from centerline
        xMat = centerline_t[:,0]
        # Grab all z points from centerline
        zMat = centerline_t[:,2]


        # compute reduced centerline coordinates
        xRed = np.matmul(self.avgMatrix,xMat)
        zRed = np.matmul(self.avgMatrix,zMat)

        # form matrix that iterates between x and z coordinates - this is the reduced centerline representation we use!
        redCenterline = np.concatenate((xRed.reshape(-1,1),zRed.reshape(-1,1)),axis=1).flatten()
        # print(redCenterline)
        # If first timestep then record centerline offset vector
        if self.step_id == 0:
            self.redCenterlineOffset = redCenterline
        ######### Get observer state estimate using reduced centerline #########
        # Get current pressure input
        u0 = np.array([pressureConstraint.value[0] for pressureConstraint in self.pressureConstraints])
        # Get current centered output
        y = redCenterline - self.redCenterlineOffset
        # Get current state estimate
        x_hat = self.stateEstimator.updateState(u0,y)

        ######### solve ROM MPC optimization for control input #########
        # Get reference trajectory in centered frame - The output is ordered such the tip of the tail is in the first spot and the tip of the head is in the last spot. ordering is [x1,z1,x2,z2,...,xn,zn]
        # y_ref = np.zeros((self.n_redCenterline*2,self.T+1))
        y_ref = self.generateReferenceTrajectory(time = t,T = self.T+1,a_max=self.ref_a_max, omega = self.ref_omega, k=self.ref_k, dt = 0.01)  #31.42 12.57
        # y_ref = y_ref - self.redCenterlineOffset.reshape(-1,1)
        pressures = self.rhcPolicy.getAction(x_hat,y_ref,pressures)



        # TODO: Save everything to database
        if self.saveOutput:
            # Save output to database
            insert_simulation_data(self.trial_id, self.step_id, t,x_hat,y_ref[:,0], pressures, redCenterline, fullState)


        # pressures = np.array([0.2,0,0.2,0,0.2,0])
        # Set pressure constraints
        for i in range(len(self.pressureConstraints)): # note! Make sure theyre ordered the right way
            self.pressureConstraints[i].value = [pressures[i]]


        # # Save verification data for debugging
        # if self.saveVerificationSet:
        #     filename = config["currentDirectory"]+"data/verificationData/"+"verification" + "_step_" + self.step_id.__str__() + ".hdf5"
        #     with h5py.File(filename, 'w') as f:
        #         f.create_dataset('k', data=self.step_id)
        #         f.create_dataset('centerline', data=centerlinePts_full)
        #         f.create_dataset('y', data=y)
        #         f.create_dataset('redCenterline', data=redCenterline)
        #         f.create_dataset('x_hat', data=x_hat)
        #         f.create_dataset('y_ref', data=y_ref)
        #         f.create_dataset('u', data=pressures)
        #         f.create_dataset('avgMatrix', data=self.avgMatrix)      
        #         f.create_dataset('t', data=t)
        # Update timestep        
        self.step_id += 1