import Sofa
import Sofa.Gui
import Sofa.Simulation
import numpy as np
import SofaRuntime
from fullStateRecorder import Exporter
from pressureConstraintController_fullBodyROMPC import PressureConstraintController_fullBodyROMPC
from pressureConstraintController import PressureConstraintController
from centerlineStateRecorderMulti import centerlineStateExporterMulti
from fullStateRecorderMulti import fullStateExporterMulti
from pressureInputRecorder import pressureInputRecorder

from rhcPolicy_ERA import rhcPolicy_ERA
# to create elements like Node or objects
import Sofa.Core
from dotenv import dotenv_values  

# Import policy 
from randomPolicy import randomPolicy
from brownianPolicy import brownianPolicy
from sinusoidalPolicy import sinusoidalPolicy

import logging

config = dotenv_values(".env")

# Choose in your script to activate or not the GUI

##################################################
# Simulation Parameters                          #
##################################################

USE_GUI = True
numSteps = 200
dt=0.001
attachPumps = False

bodyMass = 0.727 #kg
segmentMass = 2.14 #kg
coupleMass = 0.266 #kg
tailMass = 1.74 #kg
bodyModulus = 2000 #kPa
segmentModulus = 666.78 #kPa
coupleModulus = 20000 #kPa
tailModulus = 125 #kPa

segmentPoisson = 0.48
couplePoisson = 0.35
bodyPoisson = 0.35
tailPoisson = 0.42


##################################################
# Save/visualization parameters                  #
##################################################
recordPressureInputs =False
recordFullState = False
recordCenterline = False
recordInputs = False     
savePressureInputs = 0

##################################################
# Initialize control policies for each chamber   #
##################################################

policy_c00 = brownianPolicy(dt=dt, seed = 2)
policy_c01 = brownianPolicy(dt=dt, seed = 3)
policy_c10 = brownianPolicy(dt=dt, seed = 4)
policy_c11 = brownianPolicy(dt=dt, seed = 5)
policy_c20 = brownianPolicy(dt=dt, seed = 6)
policy_c21 = brownianPolicy(dt=dt, seed = 7)

sinusoidalPolicy_c00 = sinusoidalPolicy(dt=dt, phase = 180, amplitude = 0.1, frequency = 8) 
sinusoidalPolicy_c01 = sinusoidalPolicy(dt=dt, phase = 0, amplitude = 0.1, frequency = 8)
sinusoidalPolicy_c10 = sinusoidalPolicy(dt=dt, phase = 0, amplitude = 0.01, frequency = 8)
sinusoidalPolicy_c11 = sinusoidalPolicy(dt=dt, phase = 180, amplitude = 0.01, frequency = 8)
sinusoidalPolicy_c20 = sinusoidalPolicy(dt=dt, phase = 180, amplitude = 0.01, frequency = 8)
sinusoidalPolicy_c21 = sinusoidalPolicy(dt=dt, phase = 0, amplitude = 0.01, frequency = 8)

# Define bounding box for centerline

centerlineBounds = [-1200,-110, -3, 100, 110, 3] # [xmin, ymin, zmin, xmax, ymax, zmax] mm

# # Define bounding boxes for attachment points
segment0_couple0_attachmentBounds = [-295, -186, 180, -185, -15, 186] # [xmin, ymin, zmin, xmax, ymax, zmax] mm
segment1_couple0_attachmentBounds = [-295, -186, 143, -185, -15, 149] # [xmin, ymin, zmin, xmax, ymax, zmax] mm
segment1_couple1_attachmentBounds = [-295, -186, -32, -185, -15, -26] # [xmin, ymin, zmin, xmax, ymax, zmax] mm
segment2_couple1_attachmentBounds = [-295, -186, -63, -185, -15, -69] # [xmin, ymin, zmin, xmax, ymax, zmax] mm


def createScene(rootNode):
    ##################################################
    # Setup scene                                    #
    ##################################################
    rootNode.dt=dt
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
                    'SofaCUDA',
                    'Multithreading'
                    ])

    rootNode.findData('gravity').value = [0, 0, 0]
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-12, maxIterations=10000)



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
    rootNode.addObject('MeshSTLLoader', name='headVisualLoader', filename=config["currentDirectory"]+'/meshes/fullAssembly/head.stl')
    rootNode.addObject('MeshSTLLoader', name='segment0VisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/segment0.stl')
    rootNode.addObject('MeshSTLLoader', name='segment1VisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/segment1.stl')
    rootNode.addObject('MeshSTLLoader', name='segment2VisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/segment2.stl')
    rootNode.addObject('MeshSTLLoader', name='tailVisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/tail.stl')
    rootNode.addObject('MeshSTLLoader', name='couple0VisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/couple0_1.stl')
    rootNode.addObject('MeshSTLLoader', name='couple1VisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/couple1_2.stl')
    rootNode.addObject('MeshSTLLoader', name='couple2VisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/couple2_tail.stl')


    ##################################################
    # body                                           #
    ##################################################
    body = rootNode.addChild('body')
    body.addObject('MeshTopology', src='@../bodyLoader', name='bodyTopologyContainer')
    body.addObject('MechanicalObject', name='state', template='Vec3d', showObject=False, showObjectScale=1)
    body.addObject('EulerImplicit', name='odesolver')
    body.addObject('ParallelCGLinearSolver',template="ParallelCompressedRowSparseMatrixMat3x3d", name='directSolver', iterations=10, threshold=1e-15,tolerance=1e-5)
    body.addObject('LinearSolverConstraintCorrection', linearSolver='@directSolver',ODESolver='@odesolver')

    body.addObject('ParallelTetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=bodyPoisson, youngModulus=bodyModulus)
    body.addObject('UniformMass', totalMass=bodyMass)


    # ##################################################
    # # head/visual                                #
    # ##################################################
    # headVisual = body.addChild("HeadVisualModel")
    
    # headVisual.addObject('OglModel', name='visualModel', src='@../../headVisualLoader', color=[0,0.42,0, 1], updateNormals=False)
    # headVisual.addObject('BarycentricMapping')



    # ##################################################
    # # head/centerlineROI                         #
    # ##################################################    
    # # centerline_head = head.addObject('BoxROI', template="Vec3d", name="centerline_roi", box= centerlineBounds, drawBoxes=False)


    # # ################################################## 
    # # # head/constraints                               #
    # # ##################################################
    # # segment0_linearConstraint= segment0.addObject('FixedConstraint', name='fixedConstraint', indices='2905 6')
    # head_planarConstraint= head.addObject('PartialFixedConstraint', name='planarConstraint', indices='7 3',fixedDirections='1 0 1')



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


    # chamber0_0.addObject(PressureConstraintController(dt,policy_c00,savePressureInputs,chamber0_0,name="chamber0_0Controller"))

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
    rootNode.addObject(PressureConstraintController_fullBodyROMPC(dt, chambers=[chamber0_0, chamber0_1, chamber1_0, chamber1_1, chamber2_0, chamber2_1],segments = [body],saveOutput = 0))

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
    # Start logging
    logging.basicConfig(filename='testlog.log', level=logging.INFO)
    logging.info('Started')
    # # Make sure to load all SOFA libraries and plugins
    # SofaRuntime.importPlugin("SofaBaseMechanics")
    # SofaRuntime.importPlugin('SofaOpenglVisual')

    # Generate the root node
    root = Sofa.Core.Node("root")

    # Call the above function to create the scene graph
    createScene(root)

    # Once defined, initialization of the scene graph
    Sofa.Simulation.init(root)

    if not USE_GUI:
        for iteration in range(numSteps):
            logging("Iteration: " + str(iteration))
            Sofa.Simulation.animate(root, root.dt.value)
            print(iteration)
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