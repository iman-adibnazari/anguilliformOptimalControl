import Sofa
import Sofa.Gui
import Sofa.Simulation
import numpy as np
import h5py
import SofaRuntime
from fullStateRecorder import Exporter

from pressureConstraintController_fullBodyROMPC import PressureConstraintController_fullBodyROMPC
from pressureConstraintController_fullBodySysID import PressureConstraintController_fullBodySysID
from pressureConstraintController import PressureConstraintController
from centerlineStateRecorderMulti import centerlineStateExporterMulti
from fullStateRecorderMulti import fullStateExporterMulti
from pressureInputRecorder import pressureInputRecorder

from rhcPolicy_ERA import rhcPolicy_ERA
# to create elements like Node or objects
import Sofa.Core
from dotenv import dotenv_values  

import logging
import psycopg2
from psycopg2.extras import execute_values
import numpy as np
import pickle

config = dotenv_values(".env")

saveVTKs = False # Set to True to save VTK files


lowAmp = 0.02
highAmp = 0.08
allAmplitudes = [
                # [lowAmp,0,0],
				# [0,lowAmp,0],
				# [0,0,lowAmp],
				# [lowAmp,lowAmp,lowAmp],
				# [highAmp ,0,0],
				# [0,highAmp,0],
				[0,0,highAmp],
				[highAmp,highAmp,highAmp],
				]
allFrequencies = [0.1,0.3,0.5,1,1.5]
# allFrequencies = [1.5]

# Helper functions for database connection and data writing
def get_db_connection():
    conn = psycopg2.connect(
        dbname='simDB',
        user='user',
        password='password',
        host='localhost',
        port='5432'
    )
    return conn

def setup_trial(conn, trial_name, description):
    cur = conn.cursor()
    cur.execute(
        'INSERT INTO trial_metadata (trial_name, description) VALUES (%s, %s) RETURNING id',
        (trial_name, description)
    )
    trial_id = cur.fetchone()[0]
    conn.commit()
    cur.close()
    conn.close()
    return trial_id


def insert_simulation_data(conn, trial_id, timestep, simulation_time, input_data, output_data, state_data):
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
    
    conn.commit()
    cur.close()
    conn.close()


##################################################
# Connect to Database                            #
##################################################

##################################################
# Simulation Parameters                          #
##################################################
bodyMass = 9.685 #kg
bodyModulus = 2000 #kPa
segmentModulus = 666.78 #kPa
coupleModulus = 400000 #kPa
tailModulus = 125 #kPa
fr4Modulus = 25000 #kPa

segmentPoisson = 0.48
couplePoisson = 0.35
bodyPoisson = 0.35
tailPoisson = 0.42


##################################################
# Save/visualization parameters                  #
##################################################
recordPressureInputs = False
recordFullState = False
recordCenterline = False
recordInputs = False     
savePressureInputs = 0


##################################################
# bounding box for centerline                    #
##################################################
centerlineBounds = [-1200,-110, -3, 100, 110, 3] # [xmin, ymin, zmin, xmax, ymax, zmax] mm
leftHalfBounds = [-1200,-110, -300, 0, 110, -3] # [xmin, ymin, zmin, xmax, ymax, zmax] mm
rightHalfBounds = [-1200,-110, 3, 0, 110, 300] # [xmin, ymin, zmin, xmax, ymax, zmax] mm

##################################################
# bounding boxes for couples                    #
##################################################
couple0Bounds = [-354, -90, -50, -330, 90, 50] # [xmin, ymin, zmin, xmax, ymax, zmax] mm
couple1Bounds = [-568, -90, -50, -544, 90, 50] # [xmin, ymin, zmin, xmax, ymax, zmax] mm
couple2Bounds = [-781, -90, -50, -759, 90, 50] # [xmin, ymin, zmin, xmax, ymax, zmax] mm


def createScene(rootNode, expParams):
    ##################################################
    # Setup scene                                    #
    ##################################################
    rootNode.dt=expParams['dt']
    rootNode.addObject('VisualStyle', displayFlags='showVisual showForceFields showBehavior')#   
    rootNode.addObject('RequiredPlugin',
                    pluginName=['Sofa.Component.Mass',
                    'Sofa.Component.Mapping',
                    'Sofa.Component.StateContainer',
                    'Sofa.GL.Component.Rendering3D',
                    'Sofa.GL.Component.Shader',
                    'Sofa.Component.Diffusion',
                    'Sofa.Component.SolidMechanics.FEM.Elastic',
                    'Sofa.Component.IO.Mesh',
                    'Sofa.Component.Engine.Select',
                    'Sofa.Component.Engine.Transform',
                    'Sofa.Component.Mapping.MappedMatrix', 
                    'Sofa.Component.Constraint.Lagrangian.Model', 
                    'Sofa.Component.Constraint.Lagrangian.Correction', 
                    'Sofa.Component.Constraint.Lagrangian.Solver', 
                    'Sofa.Component.AnimationLoop', 
                    'Sofa.Component.Collision.Detection.Intersection', 
                    'Sofa.Component.Collision.Response.Contact',
                    'SoftRobots',
                    'SofaPython3',
                    'Sofa.Component.SolidMechanics.Spring',
                    'Sofa.Component.ODESolver.Backward',
                    'Sofa.Component.LinearSolver.Direct',
                    # 'SofaCUDA',
                    'Multithreading'
                    ])

    rootNode.findData('gravity').value = [0, 0, 0]
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=500, tolerance=1e-4)



    ##################################################
    # Load in meshes                                 #
    ##################################################
    rootNode.addObject('MeshGmshLoader', name='bodyLoader', filename=config["currentDirectory"]+'meshes/fullAssemblyContiguous/Robot_no_pump_final_contiguous.msh')
    rootNode.addObject('MeshSTLLoader', name='chamber0_0Loader', filename=config["currentDirectory"]+'/meshes/fullAssembly/chamber0_0.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber0_1Loader', filename=config["currentDirectory"]+'/meshes/fullAssembly/chamber0_1.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber1_0Loader', filename=config["currentDirectory"]+'/meshes/fullAssembly/chamber1_0.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber1_1Loader', filename=config["currentDirectory"]+'/meshes/fullAssembly/chamber1_1.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber2_0Loader', filename=config["currentDirectory"]+'/meshes/fullAssembly/chamber2_0.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber2_1Loader', filename=config["currentDirectory"]+'/meshes/fullAssembly/chamber2_1.stl')
    # rootNode.addObject('MeshSTLLoader', name='headVisualLoader', filename=config["currentDirectory"]+'/meshes/fullAssembly/head.stl')
    # rootNode.addObject('MeshSTLLoader', name='segment0VisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/segment0.stl')
    # rootNode.addObject('MeshSTLLoader', name='segment1VisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/segment1.stl')
    # rootNode.addObject('MeshSTLLoader', name='segment2VisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/segment2.stl')
    # rootNode.addObject('MeshSTLLoader', name='tailVisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/tail.stl')
    # rootNode.addObject('MeshSTLLoader', name='couple0VisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/couple0_1.stl')
    # rootNode.addObject('MeshSTLLoader', name='couple1VisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/couple1_2.stl')
    # rootNode.addObject('MeshSTLLoader', name='couple2VisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/couple2_tail.stl')


    ##################################################
    # body                                           #
    ##################################################
    body = rootNode.addChild('body')
    body.addObject('MeshTopology', src='@../bodyLoader', name='bodyTopologyContainer')
    body.addObject('MechanicalObject', name='state', template='Vec3d', showObject=False, showObjectScale=1)
    body.addObject('EulerImplicit', name='odesolver')
    body.addObject('ParallelCGLinearSolver',template="ParallelCompressedRowSparseMatrixMat3x3d", name='directSolver', iterations=250, threshold=1e-4,tolerance=1e-4, warmStart=False)
    body.addObject('LinearSolverConstraintCorrection', linearSolver='@directSolver',ODESolver='@odesolver')

    # Regions with different stiffnesses
    centerlineROI = body.addObject('BoxROI', template="Vec3d", name="centerline_roi", box= centerlineBounds, drawBoxes=True)
    leftHalfROI = body.addObject('BoxROI', template="Vec3d", name="leftHalf_roi", box= leftHalfBounds, drawBoxes=True)
    rightHalfROI = body.addObject('BoxROI', template="Vec3d", name="rightHalf_roi", box= rightHalfBounds, drawBoxes=True)
    couple0ROI = body.addObject('BoxROI', template="Vec3d", name="couple0_roi", box= couple0Bounds, drawBoxes=True)
    couple1ROI = body.addObject('BoxROI', template="Vec3d", name="couple1_roi", box= couple1Bounds, drawBoxes=True)
    couple2ROI = body.addObject('BoxROI', template="Vec3d", name="couple2_roi", box= couple2Bounds, drawBoxes=True)

    body.addObject('IndexValueMapper', template="Vec3d", name="Young1", indices="@centerline_roi.indices", value=fr4Modulus)
    body.addObject('IndexValueMapper', template="Vec3d", name="Young2", indices="@leftHalf_roi.indices", value=bodyModulus, inputValues = "@Young1.outputValues")
    body.addObject('IndexValueMapper', template="Vec3d", name="Young3", indices="@rightHalf_roi.indices", value=bodyModulus, inputValues = "@Young2.outputValues")
    body.addObject('IndexValueMapper', template="Vec3d", name="Young4", indices="@couple0_roi.indices", value=coupleModulus, inputValues = "@Young3.outputValues")
    body.addObject('IndexValueMapper', template="Vec3d", name="Young5", indices="@couple1_roi.indices", value=coupleModulus, inputValues = "@Young4.outputValues")
    body.addObject('IndexValueMapper', template="Vec3d", name="Young6", indices="@couple2_roi.indices", value=coupleModulus, inputValues = "@Young5.outputValues")

    body.addObject('ParallelTetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=bodyPoisson, youngModulus="@Young5.outputValues")
    body.addObject('UniformMass', totalMass=bodyMass)

    ################################################
    # VTKExporter                               #
    ################################################
    if saveVTKs:
        body.addObject('VTKExporter', filename=config["currentDirectory"] + f"data/visualizations/tempMeshes/Trial{expParams['trial_id']}_step", edges="0", triangles="1", quads="0", tetras="0", pointsDataFields="state.position", exportAtBegin="1", exportEveryNumberOfSteps="1")



    # ##################################################
    # # head/visual                                #
    # ##################################################
    # headVisual = body.addChild("HeadVisualModel")
    
    # headVisual.addObject('OglModel', name='visualModel', src='@../../headVisualLoader', color=[0,0.42,0, 1], updateNormals=False)
    # headVisual.addObject('BarycentricMapping')



    ##################################################
    # body/centerlineROI                         #
    ##################################################    
    # centerlineROI = body.addObject('BoxROI', template="Vec3d", name="centerline_roi", box= centerlineBounds, drawBoxes=True)
    # leftHalfROI = body.addObject('BoxROI', template="Vec3d", name="leftHalf_roi", box= leftHalfBounds, drawBoxes=True)
    # rightHalfROI = body.addObject('BoxROI', template="Vec3d", name="rightHalf_roi", box= rightHalfBounds, drawBoxes=True)



    ################################################## 
    # body/constraints                               #
    ##################################################
    # # centerline_body = body.addObject('BoxROI', template="Vec3d", name="centerline_roi", box= centerlineBounds, drawBoxes=False)
    # # body_linearConstraint= body.addObject('FixedConstraint', name='fixedConstraint', indices='7 4')
    body_planarConstraint= body.addObject('PartialFixedConstraint', name='planarConstraint', indices='18 12',fixedDirections='0 0 0')
    
    # body.addObject('RestShapeSpringsForceField', points= '18 12', stiffness=1e12, angularStiffness=100) # spring-like boundary conditions
# 13204 14531


    # # ##################################################
    # # # segment0                                       #
    # # ##################################################
    # # segment0 = rootNode.addChild('segment0')
    # # segment0.addObject('MeshTopology', src='@../segment0Loader', name='segment0TopologyContainer')
    # # segment0.addObject('MechanicalObject', name='state', template='Vec3d', showObject=False, showObjectScale=1)
    # # segment0.addObject('EulerImplicit', name='odesolver')
    # # segment0.addObject('SparseLDLSolver',template="CompressedRowSparseMatrixMat3x3d", name='directSolver')
    # # segment0.addObject('LinearSolverConstraintCorrection', linearSolver='@directSolver',ODESolver='@odesolver')

    # # segment0.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=segmentPoisson,
    # #                  youngModulus=segmentModulus)
    # # segment0.addObject('UniformMass', totalMass=segmentMass)
    
    # ##################################################
    # # segment0/visual                                #
    # ##################################################
    # segment0Visual = body.addChild("Segment0VisualModel")
    
    # segment0Visual.addObject('OglModel', name='visualModel', src='@../../segment0VisualLoader', color=[0.56,0.56,0.56, 1], updateNormals=False)
    # segment0Visual.addObject('BarycentricMapping')

    # # ##################################################
    # # # segment0/constraintLayer                       #
    # # ################################################## 

    # # ##################################################
    # # # segment0/endPlate0                             #
    # # ################################################## 

    # # ##################################################
    # # # segment0/endPlate1                             #
    # # ##################################################

    ##################################################
    # segment0/chamber0                            #
    ################################################## 
    chamber0_0 = body.addChild('chamber0_0')
    chamber0_0.addObject('MeshTopology', src='@../../chamber0_0Loader', name='chamber0_0Mesh')
    chamber0_0.addObject('MechanicalObject', name='chamber0_0',template='Vec3d')
    chamber0_0.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3d', value=1,
                     triangles='@chamber0_0Mesh.triangles', valueType='pressure')
    chamber0_0.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False, template='Vec3d,Vec3d')


    # chamber0_0.addObject(PressureConstrasrc/sims/fullAssemblyContiguousSimulation_ROMPC.pyintController(dt,policy_c00,savePressureInputs,chamber0_0,name="chamber0_0Controller"))

    ##################################################
    # segment0/chamber1                            #
    ##################################################    
    chamber0_1 = body.addChild('chamber0_1')
    chamber0_1.addObject('MeshTopology', src='@../../chamber0_1Loader', name='chamber0_1Mesh')
    chamber0_1.addObject('MechanicalObject', name='chamber0_1',template='Vec3d')
    chamber0_1.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3d', value=1,
                     triangles='@chamber0_1Mesh.triangles', valueType='pressure')
    chamber0_1.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False, template='Vec3d,Vec3d')
    

    # # chamber0_1.addObject(PressureConstraintController(dt,policy_c01,savePressureInputs,chamber0_1,name="chamber0_1Controller"))

    # # ##################################################
    # # # segment0/centerlineROI                         #
    # # ##################################################    
    # # centerline0 = segment0.addObject('BoxROI', template="Vec3d", name="centerline_roi", box= centerlineBounds, drawBoxes=False)

    # # ##################################################
    # # # segment0/segment0_couple0_attachmentROI        #
    # # ##################################################
    # # segment0_couple0_attachmentROI = segment0.addObject('BoxROI', template="Vec3d", name="segment0_couple0_attachmentROI", box= segment0_couple0_attachmentBounds, drawBoxes=False)

    # # # ##################################################
    # # # # segment0/constraints                           #
    # # # ##################################################
    # # # segment0_linearConstraint= segment0.addObject('FixedConstraint', name='fixedConstraint', indices='2905 6')
    # # # segment0_planarConstraint= segment0.addObject('PartialFixedConstraint', name='planarConstraint', indices='2907 2901 2899',fixedDirections='1 0 0')


    # # ##################################################
    # # # segment1                                       #
    # # ##################################################
    # # segment1 = rootNode.addChild('segment1')
    # # segment1.addObject('MeshTopology', src='@../segment1Loader', name='segment1TopologyContainer')
    # # segment1.addObject('MechanicalObject', name='state', template='Vec3d', showObject=False, showObjectScale=1)
    # # segment1.addObject('EulerImplicit', name='odesolver')
    # # segment1.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d", name='directSolver')
    # # segment1.addObject('LinearSolverConstraintCorrection', linearSolver='@directSolver',ODESolver='@odesolver')
    # # segment1.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=segmentPoisson,
    # #                  youngModulus=segmentModulus)
    # # segment1.addObject('UniformMass', totalMass=segmentMass)
    # ##################################################
    # # segment1/visual                                #
    # ##################################################
    # segment1Visual = body.addChild("Segment1VisualModel")
    # segment1Visual.addObject('OglModel', name='visualModel', src='@../../segment1VisualLoader', color=[0.56,0.56,0.56, 1], updateNormals=False)
    # segment1Visual.addObject('BarycentricMapping')


    # # ##################################################
    # # # segment1/constraintLayer                       #
    # # ################################################## 

    # # ##################################################
    # # # segment1/endPlate0                             #
    # # ################################################## 

    # # ##################################################
    # # # segment1/endPlate1                             #
    # # ##################################################

    ##################################################
    # segment1/chamber0                            #
    ##################################################
    chamber1_0 = body.addChild('chamber1_0')
    chamber1_0.addObject('MeshTopology', src='@../../chamber1_0Loader', name='chamber1_0Mesh')
    chamber1_0.addObject('MechanicalObject', name='chamber1_0',template='Vec3d')
    chamber1_0.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3d', value=1,
                     triangles='@chamber1_0Mesh.triangles', valueType='pressure')
    chamber1_0.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False,template='Vec3d,Vec3d')

    # # chamber1_0.addObject(PressureConstraintController(dt,policy_c10,savePressureInputs,chamber1_0,name="chamber1_0Controller"))

    ##################################################
    # segment1/chamber1                            #
    ##################################################
    chamber1_1 = body.addChild('chamber1_1')
    chamber1_1.addObject('MeshTopology', src='@../../chamber1_1Loader', name='chamber1_1Mesh')
    chamber1_1.addObject('MechanicalObject', name='chamber1_1',template='Vec3d')
    chamber1_1.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3d', value=1,
                     triangles='@chamber1_1Mesh.triangles', valueType='pressure')
    chamber1_1.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False, template='Vec3d,Vec3d')
    # # chamber1_1.addObject(PressureConstraintController(dt,policy_c11,savePressureInputs,chamber1_1,name="chamber1_1Controller"))
    
    # # ##################################################
    # # # segment1/centerlineROI                         #
    # # ##################################################  
    # # centerline1 = segment1.addObject('BoxROI', template="Vec3d", name="centerline_roi", box= centerlineBounds,drawBoxes=False)
   
    # # ##################################################
    # # # segment1/segment1_couple0_attachmentROI        #
    # # ##################################################
    # # segment1_couple0_attachmentROI = segment1.addObject('BoxROI', template="Vec3d", name="segment1_couple0_attachmentROI", box= segment1_couple0_attachmentBounds, drawBoxes=False)

    # # ##################################################
    # # # segment1/segment1_couple1_attachmentROI        #
    # # ##################################################
    # # segment1_couple1_attachmentROI = segment1.addObject('BoxROI', template="Vec3d", name="segment1_couple1_attachmentROI", box= segment1_couple1_attachmentBounds, drawBoxes=False)

    # # ##################################################
    # # # segment2                                       #
    # # ##################################################
    # # segment2 = rootNode.addChild('segment2')
    # # segment2.addObject('MeshTopology', src='@../segment2Loader', name='segment2TopologyContainer')
    # # segment2.addObject('MechanicalObject', name='state', template='Vec3d', showObject=False, showObjectScale=1)
    # # segment2.addObject('EulerImplicit', name='odesolver')
    # # segment2.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d", name='directSolver')
    # # segment2.addObject('LinearSolverConstraintCorrection', linearSolver='@directSolver',ODESolver='@odesolver')
    # # segment2.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=segmentPoisson,
    # #                  youngModulus=segmentModulus)
    # # segment2.addObject('UniformMass', totalMass=segmentMass)
    # ##################################################
    # # segment2/visual                                #
    # ##################################################
    # segment2Visual = body.addChild("Segment2VisualModel")
    # segment2Visual.addObject('OglModel', name='visualModel', src='@../../segment2VisualLoader', color=[0.56,0.56,0.56, 1], updateNormals=False)
    # segment2Visual.addObject('BarycentricMapping')

    # # ##################################################
    # # # segment2/constraintLayer                       #
    # # ################################################## 

    # # ##################################################
    # # # segment2/endPlate0                             #
    # # ################################################## 

    # # ##################################################
    # # # segment2/endPlate1                             #
    # # ##################################################

    ##################################################
    # segment2/chamber0                              #
    ##################################################
    chamber2_0 = body.addChild('chamber2_0')
    chamber2_0.addObject('MeshTopology', src='@../../chamber2_0Loader', name='chamber2_0Mesh')
    chamber2_0.addObject('MechanicalObject', name='chamber2_0',template='Vec3d')
    chamber2_0.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3d', value=1,
                     triangles='@chamber2_0Mesh.triangles', valueType='pressure')
    chamber2_0.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False, template='Vec3d,Vec3d')

    # # chamber2_0.addObject(PressureConstraintController(dt,policy_c20,savePressureInputs,chamber2_0,name="chamber2_0Controller"))

    ##################################################
    # segment2/chamber1                            #
    ##################################################
    chamber2_1 = body.addChild('chamber2_1')
    chamber2_1.addObject('MeshTopology', src='@../../chamber2_1Loader', name='chamber2_1Mesh')
    chamber2_1.addObject('MechanicalObject', name='chamber2_1',template='Vec3d')
    chamber2_1.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3d', value=1,
                     triangles='@chamber2_1Mesh.triangles', valueType='pressure')
    chamber2_1.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False, template='Vec3d,Vec3d')
    
    # # chamber2_1.addObject(PressureConstraintController(dt,policy_c21,savePressureInputs,chamber2_1,name="chamber2_1Controller"))
    
    # # ##################################################
    # # # segment2/centerlineROI                         #
    # # ##################################################  
    # # centerline2 = segment2.addObject('BoxROI', template="Vec3d", name="centerline_roi", box= centerlineBounds,drawBoxes=False)

    # # ##################################################
    # # # Setup up state estimator and control policy    #
    # # ##################################################


    # ##################################################
    # # Attach pressure controller                     #
    # ##################################################
    rootNode.addObject(PressureConstraintController_fullBodyROMPC(expParams['dt'], chambers=[chamber0_0, chamber0_1, chamber1_0, chamber1_1, chamber2_0, chamber2_1],segments = [body],expParams=expParams, saveOutput = True))

    # ##################################################
    # # Set up data recording                          #
    # ##################################################

    # if recordCenterline: 
    #     rootNode.addObject(centerlineStateExporterMulti(filetype=0, name='centerlineExporter', segments = [head, segment0, segment1,segment2, tail]))
    # if recordFullState: 
    #     rootNode.addObject(fullStateExporterMulti(filetype=0, name='stateExporter', segments = [head, segment0, segment1,segment2, tail]))
    # if recordPressureInputs: 
    #     rootNode.addObject(pressureInputRecorder(name='inputExporter', segments = [segment0, segment1, segment2]))


def main():
    TimeHorizon = 10 # simulation length (seconds)
    speedups = [1] #1, 2, 5, 10
    USE_GUI = False
    ref_a_max = 10  # mm 
    ref_omega = (6.28*0.9)
    ref_k=(6.28*0.8)
    trainingTrialInd = 0
    isTrainingTrial = False
    isPhysRobotTrial = True
    physRobotTrialInds = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]# 
    trialParamString = ''
    romName = "eraSystemMatricesAndGains_10dim_1train"
    for trialInd in physRobotTrialInds:
        #     for amplitudes in allAmplitudes:
        #         for freq in allFrequencies:
        # Generate the root node
        root = Sofa.Core.Node("root")
        # Call the above function to create the scene graph
        # Connect to the database
        conn = get_db_connection()
        dt = 0.01
        numSteps = int(TimeHorizon/dt)
        # Save Trial MetaData in Database
        if isPhysRobotTrial:
            # Read in trial params from h5 file
            physExpsFilename = config["currentDirectory"]+"data/archivedDataSets/ContiguousAssembly/experimentalRefTrajectories.h5"
            # Load in h5 file
            with h5py.File(physExpsFilename, 'r') as f:
                trialParamString = f['trialParameters'][:].astype(str)[trialInd,0]
            print(trialParamString)
        trial_name = f"physRobotTracking_{trialParamString}: {romName}"
        # put experimental parameters in description
        if USE_GUI:
            if isTrainingTrial:
                description = f"ControlTrial: Model-{romName}, refTrajParams: feasibleTrajectoryTrialNum-{trainingTrialInd}"
            elif isPhysRobotTrial:
                description = f"ControlTrial: Model-{romName}, refTrajParams: physRobotTrialInd-{trialInd}, params-{trialParamString}"
            else:
                description = f"ControlTrial: Model-{romName}, refTrajParams: a_max-{ref_a_max}, omega-{ref_omega}, k-{ref_k}" #f"dt: {dt}, amplitudes: {amplitudes}, frequencies: {frequencies}, phases: {phases}, numSteps: {numSteps}"
        else:
            if isTrainingTrial:
                description = f"ControlTrial: Model-{romName}, refTrajParams: feasibleTrajectoryTrialNum-{trainingTrialInd}"
            elif isPhysRobotTrial:
                description = f"ControlTrial: Model-{romName}, refTrajParams: physRobotTrialInd-{trialInd}, params-{trialParamString}"
            else:
                description = f"ControlTrial: Model-{romName}, refTrajParams: a_max-{ref_a_max}, omega-{ref_omega}, k-{ref_k}" #f"dt: {dt}, amplitudes: {amplitudes}, frequencies: {frequencies}, phases: {phases}, numSteps: {numSteps}"
        trial_id = setup_trial(conn, trial_name, description)
        print(f"New trial created with ID: {trial_id}")
        # Stuff experiment parameters and metadata into dictionary
        expParams = {"dt": dt, "trial_id": trial_id, "conn": conn, "ref_a_max": ref_a_max, "ref_omega": ref_omega, "ref_k": ref_k, "modelName": romName, "isTrainingTrial": isTrainingTrial, "trainingTrialInd": trainingTrialInd, "isPhysRobotTrial": isPhysRobotTrial, "physRobotTrialInd": trialInd}
        createScene(root, expParams)

        # Once defined, initialization of the scene graph
        Sofa.Simulation.init(root)



        if not USE_GUI:
            for iteration in range(numSteps):
                # logging("Iteration: " + str(iteration))
                Sofa.Simulation.animate(root, root.dt.value)
                print(f"dt: {dt}")#, amplitudes: {amplitudes}, frequencies: {frequencies}, phases: {phases}, Iteration: {iteration} out of {numSteps}")
        else:
            # Find out the supported GUIs
            print ("Supported GUIs are: " + Sofa.Gui.GUIManager.ListSupportedGUI(","))
            # Launch the GUI (qt or qglviewer)
            Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
            Sofa.Gui.GUIManager.createGUI(root, __file__)
            Sofa.Gui.GUIManager.SetDimension(1080, 1080)
            # Initialization of the scene will be done here
            Sofa.Gui.GUIManager.MainLoop(root)
            Sofa.Gui.GUIManager.closeGUI()
            print("GUI was closed")

        print("Simulation is done.")

# Function used only if this script is called from a python environment, triggers the main()
if __name__ == '__main__':
    main()