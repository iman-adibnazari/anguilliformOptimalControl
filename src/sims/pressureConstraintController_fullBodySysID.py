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


def insert_simulation_data(trial_id, timestep, simulation_time, input_data, output_data, state_data):
    conn = get_db_connection()
    cur = conn.cursor()
    
    # Serialize NumPy arrays
    input_data_bin = pickle.dumps(input_data)
    output_data_bin = pickle.dumps(output_data)
    state_data_bin = pickle.dumps(state_data)
    
    # Insert data into the database
    cur.execute(
        '''
        INSERT INTO simulation_data 
        (trial_id, timestep, simulation_time, input_data, output_data, state_data) 
        VALUES (%s, %s, %s, %s, %s, %s)
        ''',
        (trial_id, timestep, simulation_time, psycopg2.Binary(input_data_bin), psycopg2.Binary(output_data_bin), psycopg2.Binary(state_data_bin))
    )
    print("Inserted timestep "+timestep.__str__() + " into database")
    conn.commit()
    cur.close()
    conn.close()

class PressureConstraintController_fullBodySysID(Sofa.Core.Controller):
    def __init__(self,deltaT,chambers,segments, saveOutput, expParams, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.step_id = 0 
        self.length = 1114.1947932504659 # mm
        self.T = 1300 # prediction horizon
        self.deltaT = deltaT
        self.chambers = chambers
        self.saveOutput=saveOutput
        self.segments = segments
        self.expParams = expParams
        self.rhcPolicy =  centralizedPolicy_sinusoid(deltaT,T=self.T, amplitudes = expParams["amplitudes"], frequencies = expParams["frequencies"], phases = expParams["phases"])
        self.pressureConstraints = [chamber.getObject('SurfacePressureConstraint') for chamber in self.chambers]
        self.conn = get_db_connection()
        self.trial_id = expParams["trial_id"]
        for pressureConstraint in self.pressureConstraints:
            pressureConstraint.value = [0]
        
        # parameters for centerline reduction
        self.n_redCenterline = 20 # number of discretized points in reduced centerline
        self.n_redLocal = 20 # number of points to average over closest to the discretized points
        self.avgMatrix = np.zeros((self.n_redCenterline,self.n_redLocal))
        self.redCenterlineOffset = np.zeros((self.n_redCenterline*2)) 
        print(self.name.getValueString().__str__())
        self.saveVerificationSet = True


    def onAnimateBeginEvent(self, e):
        t=self.step_id*self.deltaT # get current time step
        pressures = np.zeros(len(self.pressureConstraints))

        pressures = self.rhcPolicy.getAction(0,0 ,pressures)
        print("Time = "+t.__str__() + " Pressures = "+pressures.__str__())
        # Grab Current Full Order State and Output
        # Measure time elapsed for collecting state and output data
        start = time.time()
        # Grab Centerline
        # get full centerline vector
        centerlinePts_full = np.empty((1,3))
        for ind,segment in enumerate(self.segments): 

            with segment.state.position.writeableArray() as wa:
                if ind ==0:
                    fullState = wa 
                else: 
                    fullState = np.concatenate((x,wa))
            with segment.centerline_roi.indices.writeableArray() as indices:

                with segment.state.position.writeableArray() as wa:
                    temp = wa[indices,:]
                    if ind ==0:
                        centerlinePts_full = temp 
                    else: 
                        centerlinePts_full = np.concatenate((centerlinePts_full,temp))
        # Check time for collecting state and output data
        end1 = time.time()
        
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


        # Check time for computing reduced centerline
        end2 = time.time()
    

        # TODO: Save everything to database
        if self.saveOutput:
            # Save output to database
            insert_simulation_data(self.trial_id, self.step_id, t, pressures, redCenterline, fullState)
    
        # Check time for saving output
        end3 = time.time()
        print("Time for collecting state and output data: ", end1 - start)
        print("Time for computing reduced centerline: ", end2 - end1)
        print("Time for saving output: ", end3 - end2)
        # update pressure constraints

        for i in range(len(self.pressureConstraints)): # note! Make sure theyre ordered the right way
            self.pressureConstraints[i].value = [pressures[i]]

    

        # update time step
        self.step_id += 1