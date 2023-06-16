import Sofa
import Sofa.Gui
import Sofa.Simulation
import numpy as np
import SofaRuntime
from fullStateRecorder import Exporter
from pressureConstraintController import PressureConstraintController
from centerlineStateRecorderMulti import centerlineStateExporterMulti
from fullStateRecorderMulti import fullStateExporterMulti
from pressureInputRecorder import pressureInputRecorder

# to create elements like Node or objects
import Sofa.Core
from dotenv import dotenv_values  

# Import policy 
from randomPolicy import randomPolicy
from brownianPolicy import brownianPolicy
from sinusoidalPolicy import sinusoidalPolicy



config = dotenv_values(".env")

# Choose in your script to activate or not the GUI

##################################################
# Simulation Parameters                          #
##################################################
USE_GUI = True
numSteps = 200
dt=0.001
attachPumps = False
segmentMass = 1 #kg
coupleMass = 0.1


##################################################
# Save/visualization parameters                  #
##################################################
recordPressureInputs =True
recordFullState = True
recordCenterline = True
recordInputs = True     
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

centerlineBounds = [-243,-184, -254, -237, -15, 370] # [xmin, ymin, zmin, xmax, ymax, zmax] mm

# Define bounding boxes for attachment points
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
                    pluginName='SoftRobots SofaPython3 SofaLoader SofaSimpleFem SofaEngine SofaDeformable SofaImplicitOdeSolver SofaConstraint SofaSparseSolver')

    rootNode.findData('gravity').value = [0, -9.81, 0]
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-12, maxIterations=10000)



    ##################################################
    # Load in meshes                                 #
    ##################################################
    rootNode.addObject('MeshGmshLoader', name='segment0Loader', filename=config["currentDirectory"]+'meshes/threeSegments/segment0.msh')
    rootNode.addObject('MeshGmshLoader', name='segment1Loader', filename=config["currentDirectory"]+'meshes/threeSegments/segment1.msh')
    rootNode.addObject('MeshGmshLoader', name='segment2Loader', filename=config["currentDirectory"]+'meshes/threeSegments/segment2.msh')
    # rootNode.addObject('MeshSTLLoader', name='segment0VisualLoader', filename=config["currentDirectory"]+'meshes/threeSegments/segment0.stl')
    # rootNode.addObject('MeshSTLLoader', name='segment1VisualLoader', filename=config["currentDirectory"]+'meshes/threeSegments/segment1.stl')
    # rootNode.addObject('MeshSTLLoader', name='segment2VisualLoader', filename=config["currentDirectory"]+'meshes/threeSegments/segment2.stl')
    rootNode.addObject('MeshGmshLoader', name='couple0Loader', filename=config["currentDirectory"]+'meshes/threeSegments/couple0_1.msh')
    rootNode.addObject('MeshGmshLoader', name='couple1Loader', filename=config["currentDirectory"]+'meshes/threeSegments/couple1_2.msh')
    rootNode.addObject('MeshSTLLoader', name='chamber0_0Loader', filename=config["currentDirectory"]+'/meshes/threeSegments/chamber0_0.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber0_1Loader', filename=config["currentDirectory"]+'/meshes/threeSegments/chamber0_1.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber1_0Loader', filename=config["currentDirectory"]+'/meshes/threeSegments/chamber1_0.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber1_1Loader', filename=config["currentDirectory"]+'/meshes/threeSegments/chamber1_1.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber2_0Loader', filename=config["currentDirectory"]+'/meshes/threeSegments/chamber2_0.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber2_1Loader', filename=config["currentDirectory"]+'/meshes/threeSegments/chamber2_1.stl')

    ##################################################
    # segment0                                       #
    ##################################################
    segment0 = rootNode.addChild('segment0')
    segment0.addObject('MeshTopology', src='@../segment0Loader', name='segment0TopologyContainer')
    segment0.addObject('MechanicalObject', name='state', template='Vec3', showObject=False, showObjectScale=1)
    segment0.addObject('EulerImplicit', name='odesolver')
    segment0.addObject('SparseLDLSolver', name='directSolver')
    segment0.addObject('LinearSolverConstraintCorrection', solverName='directSolver')

    segment0.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=1000)
    segment0.addObject('UniformMass', totalMass=segmentMass)
    
    # ##################################################
    # # segment0/visual                                #
    # ##################################################
    # segment0Visual = segment0.addChild("VisualModel")
    
    # segment0Visual.addObject('OglModel', name='visualModel', src='@../../segment0VisualLoader', color=[0.5,0.5,0.5, 0.5], updateNormals=False)
    # segment0Visual.addObject('BarycentricMapping')

    ##################################################
    # segment0/constraintLayer                       #
    ################################################## 

    ##################################################
    # segment0/endPlate0                             #
    ################################################## 

    ##################################################
    # segment0/endPlate1                             #
    ##################################################

    ##################################################
    # segment0/chamber0                            #
    ################################################## 
    chamber0_0 = segment0.addChild('chamber0')
    chamber0_0.addObject('MeshTopology', src='@../../chamber0_0Loader', name='chamber0_0Mesh')
    chamber0_0.addObject('MechanicalObject', name='chamber0_0')
    chamber0_0.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@chamber0_0Mesh.triangles', valueType='pressure')
    chamber0_0.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)


    chamber0_0.addObject(PressureConstraintController(dt,policy_c00,savePressureInputs,chamber0_0,name="chamber0_0Controller"))

    ##################################################
    # segment0/chamber1                            #
    ##################################################    
    chamber0_1 = segment0.addChild('chamber1')
    chamber0_1.addObject('MeshTopology', src='@../../chamber0_1Loader', name='chamber0_1Mesh')
    chamber0_1.addObject('MechanicalObject', name='chamber0_1')
    chamber0_1.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@chamber0_1Mesh.triangles', valueType='pressure')
    chamber0_1.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)
    

    chamber0_1.addObject(PressureConstraintController(dt,policy_c01,savePressureInputs,chamber0_1,name="chamber0_1Controller"))

    ##################################################
    # segment0/centerlineROI                         #
    ##################################################    
    centerline0 = segment0.addObject('BoxROI', template="Vec3d", name="centerline_roi", box= centerlineBounds, drawBoxes=True)

    ##################################################
    # segment0/segment0_couple0_attachmentROI        #
    ##################################################
    segment0_couple0_attachmentROI = segment0.addObject('BoxROI', template="Vec3d", name="segment0_couple0_attachmentROI", box= segment0_couple0_attachmentBounds, drawBoxes=True)

    ##################################################
    # segment1                                       #
    ##################################################
    segment1 = rootNode.addChild('segment1')
    segment1.addObject('MeshTopology', src='@../segment1Loader', name='segment1TopologyContainer')
    segment1.addObject('MechanicalObject', name='state', template='Vec3', showObject=False, showObjectScale=1)
    segment1.addObject('EulerImplicit', name='odesolver')
    segment1.addObject('SparseLDLSolver', name='directSolver')
    segment1.addObject('LinearSolverConstraintCorrection', solverName='directSolver')
    segment1.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=1000)
    segment1.addObject('UniformMass', totalMass=segmentMass)
    # ##################################################
    # # segment1/visual                                #
    # ##################################################
    # segment1Visual = segment1.addChild("VisualModel")
    # segment1Visual.addObject('OglModel', name='visualModel', src='@../../segment1VisualLoader', color=[0.5,0.5,0.5, 0.5], updateNormals=False)
    # segment1Visual.addObject('BarycentricMapping')


    ##################################################
    # segment1/constraintLayer                       #
    ################################################## 

    ##################################################
    # segment1/endPlate0                             #
    ################################################## 

    ##################################################
    # segment1/endPlate1                             #
    ##################################################

    ##################################################
    # segment1/chamber0                            #
    ##################################################
    chamber1_0 = segment1.addChild('chamber0')
    chamber1_0.addObject('MeshTopology', src='@../../chamber1_0Loader', name='chamber1_0Mesh')
    chamber1_0.addObject('MechanicalObject', name='chamber1_0')
    chamber1_0.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@chamber1_0Mesh.triangles', valueType='pressure')
    chamber1_0.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

    chamber1_0.addObject(PressureConstraintController(dt,policy_c10,savePressureInputs,chamber1_0,name="chamber1_0Controller"))

    ##################################################
    # segment1/chamber1                            #
    ##################################################
    chamber1_1 = segment1.addChild('chamber1')
    chamber1_1.addObject('MeshTopology', src='@../../chamber1_1Loader', name='chamber1_1Mesh')
    chamber1_1.addObject('MechanicalObject', name='chamber1_1')
    chamber1_1.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@chamber1_1Mesh.triangles', valueType='pressure')
    chamber1_1.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)
    chamber1_1.addObject(PressureConstraintController(dt,policy_c11,savePressureInputs,chamber1_1,name="chamber1_1Controller"))
    
    ##################################################
    # segment1/centerlineROI                         #
    ##################################################  
    centerline1 = segment1.addObject('BoxROI', template="Vec3d", name="centerline_roi", box= centerlineBounds,drawBoxes=True)
   
    ##################################################
    # segment1/segment1_couple0_attachmentROI        #
    ##################################################
    segment1_couple0_attachmentROI = segment1.addObject('BoxROI', template="Vec3d", name="segment1_couple0_attachmentROI", box= segment1_couple0_attachmentBounds, drawBoxes=True)

    ##################################################
    # segment1/segment1_couple1_attachmentROI        #
    ##################################################
    segment1_couple1_attachmentROI = segment1.addObject('BoxROI', template="Vec3d", name="segment1_couple1_attachmentROI", box= segment1_couple1_attachmentBounds, drawBoxes=True)

    ##################################################
    # segment2                                       #
    ##################################################
    segment2 = rootNode.addChild('segment2')
    segment2.addObject('MeshTopology', src='@../segment2Loader', name='segment2TopologyContainer')
    segment2.addObject('MechanicalObject', name='state', template='Vec3', showObject=False, showObjectScale=1)
    segment2.addObject('EulerImplicit', name='odesolver')
    segment2.addObject('SparseLDLSolver', name='directSolver')
    segment2.addObject('LinearSolverConstraintCorrection', solverName='directSolver')
    segment2.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=1000)
    segment2.addObject('UniformMass', totalMass=segmentMass)
    # ##################################################
    # # segment2/visual                                #
    # ##################################################
    # segment2Visual = segment1.addChild("VisualModel")
    # segment2Visual.addObject('OglModel', name='visualModel', src='@../../segment2VisualLoader', color=[0.5,0.5,0.5, 0.5], updateNormals=False)
    # segment2Visual.addObject('BarycentricMapping')


    ##################################################
    # segment2/constraintLayer                       #
    ################################################## 

    ##################################################
    # segment2/endPlate0                             #
    ################################################## 

    ##################################################
    # segment2/endPlate1                             #
    ##################################################

    ##################################################
    # segment2/chamber0                            #
    ##################################################
    chamber2_0 = segment2.addChild('chamber0')
    chamber2_0.addObject('MeshTopology', src='@../../chamber2_0Loader', name='chamber2_0Mesh')
    chamber2_0.addObject('MechanicalObject', name='chamber2_0')
    chamber2_0.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@chamber2_0Mesh.triangles', valueType='pressure')
    chamber2_0.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

    chamber2_0.addObject(PressureConstraintController(dt,policy_c20,savePressureInputs,chamber2_0,name="chamber2_0Controller"))

    ##################################################
    # segment2/chamber1                            #
    ##################################################
    chamber2_1 = segment2.addChild('chamber1')
    chamber2_1.addObject('MeshTopology', src='@../../chamber2_1Loader', name='chamber2_1Mesh')
    chamber2_1.addObject('MechanicalObject', name='chamber2_1')
    chamber2_1.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@chamber2_1Mesh.triangles', valueType='pressure')
    chamber2_1.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)


    chamber2_1.addObject(PressureConstraintController(dt,policy_c21,savePressureInputs,chamber2_1,name="chamber2_1Controller"))
    
    ##################################################
    # segment2/centerlineROI                         #
    ##################################################  
    centerline2 = segment2.addObject('BoxROI', template="Vec3d", name="centerline_roi", box= centerlineBounds,drawBoxes=True)

    ##################################################
    # segment2/segment2_couple1_attachmentROI        #
    ##################################################
    segment2_couple1_attachmentROI = segment2.addObject('BoxROI', template="Vec3d", name="segment2_couple1_attachmentROI", box= segment2_couple1_attachmentBounds, drawBoxes=True)

    ##################################################
    # couple0                                        #
    ##################################################
    couple0 = rootNode.addChild('couple0')
    couple0.addObject('MeshTopology', src='@../couple0Loader', name='couple0TopologyContainer')
    couple0.addObject('MechanicalObject', name='state', template='Vec3', showObject=False, showObjectScale=1)

    couple0.addObject('EulerImplicit', name='odesolver')
    couple0.addObject('SparseLDLSolver', name='directSolver')
    couple0.addObject('LinearSolverConstraintCorrection', solverName='directSolver')
    couple0.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=1000000)

    couple0.addObject('UniformMass', totalMass=coupleMass)


    # ##################################################
    # # couple0/visual                                 #
    # ##################################################
    # couple0Visual = couple0.addChild("VisualModel")

    # couple0Visual.addObject('OglModel', name='visualModel', src='@../../couple0Loader', color=[0.7,0.7,1, 0.5], updateNormals=False)
    # couple0Visual.addObject('BarycentricMapping')

    ##################################################
    # couple0/couple0_segment0_attachmentROI         #
    ##################################################
    couple0_segment0_attachmentROI = couple0.addObject('BoxROI', template="Vec3d", name="segment0_couple0_attachmentROI", box= segment0_couple0_attachmentBounds, drawBoxes=True)

    ##################################################
    # couple0/couple0_segment1_attachmentROI         #
    ##################################################
    couple0_segment1_attachmentROI = couple0.addObject('BoxROI', template="Vec3d", name="segment1_couple0_attachmentROI", box= segment1_couple0_attachmentBounds, drawBoxes=True)

    ##################################################
    # couple1                                        #
    ##################################################
    couple1 = rootNode.addChild('couple1')
    couple1.addObject('MeshTopology', src='@../couple1Loader', name='couple1TopologyContainer')
    couple1.addObject('MechanicalObject', name='state', template='Vec3', showObject=False, showObjectScale=1)

    couple1.addObject('EulerImplicit', name='odesolver')
    couple1.addObject('SparseLDLSolver', name='directSolver')
    couple1.addObject('LinearSolverConstraintCorrection', solverName='directSolver')
    couple1.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=1000000)

    couple1.addObject('UniformMass', totalMass=coupleMass)


    # ##################################################
    # # couple1/visual                                 #
    # ##################################################
    # couple1Visual = couple0.addChild("VisualModel")

    # couple1Visual.addObject('OglModel', name='visualModel', src='@../../couple1Loader', color=[0.7,0.7,1, 0.5], updateNormals=False)
    # couple1Visual.addObject('BarycentricMapping')

    ##################################################
    # couple1/couple1_segment1_attachmentROI         #
    ##################################################
    couple1_segment1_attachmentROI = couple1.addObject('BoxROI', template="Vec3d", name="segment1_couple1_attachmentROI", box= segment1_couple1_attachmentBounds, drawBoxes=True)

    ##################################################
    # couple1/couple1_segment2_attachmentROI         #
    ##################################################
    couple1_segment2_attachmentROI = couple1.addObject('BoxROI', template="Vec3d", name="segment2_couple1_attachmentROI", box= segment2_couple1_attachmentBounds, drawBoxes=True)


    ##################################################
    # attach segment0 to couple0                     #
    ##################################################
    # rootNode.addObject('AttachConstraint', object1="@segment0", object2="@couple0", indices1="2887 2886 2885 2888", indices2="18 19 20 17", constraintFactor="1 1 1 1")
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@couple0', first_point='2886', second_point='17')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@couple0', first_point='2885', second_point='18')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@couple0', first_point='2884', second_point='19')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@couple0', first_point='2887', second_point='16')    

    ##################################################
    # attach segment1 to couple0                     #
    ##################################################
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple0', object2='@segment1', first_point='22', second_point='19')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple0', object2='@segment1', first_point='21', second_point='16')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple0', object2='@segment1', first_point='23', second_point='17')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple0', object2='@segment1', first_point='20', second_point='18')    
    
    
    
    ##################################################
    # attach segment1 to couple1                     #
    ################################################## 
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@couple1', first_point='2886', second_point='17')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@couple1', first_point='2885', second_point='18')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@couple1', first_point='2884', second_point='19')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@couple1', first_point='2887', second_point='16')    

    ##################################################
    # attach segment2 to couple1                     #
    ##################################################
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple1', object2='@segment2', first_point='22', second_point='19')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple1', object2='@segment2', first_point='21', second_point='16')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple1', object2='@segment2', first_point='23', second_point='17')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple1', object2='@segment2', first_point='20', second_point='18')    



    if recordCenterline: 
        rootNode.addObject(centerlineStateExporterMulti(filetype=0, name='centerlineExporter', segments = [segment0, segment1,segment2]))
    if recordFullState: 
        rootNode.addObject(fullStateExporterMulti(filetype=0, name='stateExporter', segments = [segment0, segment1,segment2]))
    if recordPressureInputs: 
        rootNode.addObject(pressureInputRecorder(name='inputExporter', segments = [segment0, segment1, segment2]))



    # if attachPumps: 

    #     ##################################################
    #     # pump0                                          #
    #     ##################################################
    #     pump0 = rootNode.addChild('pump0')
    #     pump0.addObject('MeshTopology', src='@../pump0Loader', name='pump0TopologyContainer')
    #     pump0.addObject('MechanicalObject', name='state', template='Vec3', showObject=False, showObjectScale=1)
    #     pump0.addObject('EulerImplicit', name='odesolver')
    #     pump0.addObject('SparseLDLSolver', name='directSolver')
    #     pump0.addObject('LinearSolverConstraintCorrection', solverName='directSolver')
    #     pump0.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
    #                     youngModulus=100000)
    #     pump0.addObject('UniformMass', totalMass=0.00008)

    #     ##################################################
    #     # pump0/visual                                   #
    #     ##################################################
    #     pump0Visual = pump0.addChild("VisualModel")
    #     pump0Visual.addObject('OglModel', name='visualModel', src='@../../pump0Loader', color=[0.7,1,0.7, 0.5], updateNormals=False)
    #     pump0Visual.addObject('BarycentricMapping')
    #     ##################################################
    #     # pump1                                          #
    #     ##################################################
    #     pump1 = rootNode.addChild('pump1')
    #     pump1.addObject('MeshTopology', src='@../pump1Loader', name='pump1TopologyContainer')
    #     pump1.addObject('MechanicalObject', name='state', template='Vec3', showObject=False, showObjectScale=1)
    #     pump1.addObject('EulerImplicit', name='odesolver')
    #     pump1.addObject('SparseLDLSolver', name='directSolver')
    #     pump1.addObject('LinearSolverConstraintCorrection', solverName='directSolver')
    #     pump1.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
    #                     youngModulus=100000)
    #     pump1.addObject('UniformMass', totalMass=0.00008)

    #     ##################################################
    #     # pump1/visual                                   #
    #     ##################################################
    #     pump1Visual = pump1.addChild("VisualModel")
    #     pump1Visual.addObject('OglModel', name='visualModel', src='@../../pump1Loader', color=[0.7,1,0.7, 0.5], updateNormals=False)
    #     pump1Visual.addObject('BarycentricMapping')






    #     ##################################################
    #     # attach segment0 to pump0                       #
    #     ##################################################
    #     rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@pump0', first_point='1638', second_point='22')    
    #     rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@pump0', first_point='618', second_point='42')    
    #     rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@pump0', first_point='3059', second_point='187')    



    #     ##################################################
    #     # attach segment1 to pump1                       #
    #     ##################################################
    #     rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@pump1', first_point='1870', second_point='4')    
    #     rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@pump1', first_point='625', second_point='9')    
    #     rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@pump1', first_point='624', second_point='32')    
    #     rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@pump1', first_point='3865', second_point='174')    



def main():
    # Make sure to load all SOFA libraries and plugins
    SofaRuntime.importPlugin("SofaBaseMechanics")
    SofaRuntime.importPlugin('SofaOpenglVisual')

    # Generate the root node
    root = Sofa.Core.Node("root")

    # Call the above function to create the scene graph
    createScene(root)

    # Once defined, initialization of the scene graph
    Sofa.Simulation.init(root)

    if not USE_GUI:
        for iteration in range(numSteps):
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