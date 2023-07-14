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



config = dotenv_values(".env")

# Choose in your script to activate or not the GUI

##################################################
# Simulation Parameters                          #
##################################################
USE_GUI = False
numSteps = 2000
numEpisodes = 15
dt=0.001
attachPumps = False
headMass = 1 #kg
segmentMass = 1 #kg
coupleMass = 0.1 #kg
tailMass = 1 #kg


##################################################
# Save/visualization parameters                  #
##################################################
recordPressureInputs =True
recordFullState = True
recordCenterline = True
recordInputs = True     
savePressureInputs = 0





def createScene(rootNode, policySeed = 0):
    ##################################################
    # Setup scene                                    #
    ##################################################
    rootNode.dt=dt
    rootNode.addObject('VisualStyle', displayFlags='showVisual') # showForceFields showBehavior 
    rootNode.addObject('RequiredPlugin',
                    pluginName='SoftRobots SofaPython3 SofaLoader SofaSimpleFem SofaEngine SofaDeformable SofaImplicitOdeSolver SofaConstraint SofaSparseSolver')

    rootNode.findData('gravity').value = [0, 0, 0]
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-12, maxIterations=10000)


    ##################################################
    # Initialize control policies for each chamber   #
    ##################################################

    policy_c00 = brownianPolicy(dt=dt, seed = policySeed)
    policy_c01 = brownianPolicy(dt=dt, seed = policySeed+5)
    policy_c10 = brownianPolicy(dt=dt, seed = policySeed+6)
    policy_c11 = brownianPolicy(dt=dt, seed = policySeed+7)
    policy_c20 = brownianPolicy(dt=dt, seed = policySeed+8)
    policy_c21 = brownianPolicy(dt=dt, seed = policySeed+9)


    # Define bounding box for centerline

    centerlineBounds = [-1200,-110, -3, 1, 110, 3] # [xmin, ymin, zmin, xmax, ymax, zmax] mm

    # Define bounding boxes for attachment points
    segment0_couple0_attachmentBounds = [-295, -186, 180, -185, -15, 186] # [xmin, ymin, zmin, xmax, ymax, zmax] mm
    segment1_couple0_attachmentBounds = [-295, -186, 143, -185, -15, 149] # [xmin, ymin, zmin, xmax, ymax, zmax] mm
    segment1_couple1_attachmentBounds = [-295, -186, -32, -185, -15, -26] # [xmin, ymin, zmin, xmax, ymax, zmax] mm
    segment2_couple1_attachmentBounds = [-295, -186, -63, -185, -15, -69] # [xmin, ymin, zmin, xmax, ymax, zmax] mm


    ##################################################
    # Load in meshes                                 #
    ##################################################
    rootNode.addObject('MeshGmshLoader', name='headLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/head.msh')
    rootNode.addObject('MeshGmshLoader', name='segment0Loader', filename=config["currentDirectory"]+'meshes/fullAssembly/segment0.msh')
    rootNode.addObject('MeshGmshLoader', name='segment1Loader', filename=config["currentDirectory"]+'meshes/fullAssembly/segment1.msh')
    rootNode.addObject('MeshGmshLoader', name='segment2Loader', filename=config["currentDirectory"]+'meshes/fullAssembly/segment2.msh')
    rootNode.addObject('MeshGmshLoader', name='tailLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/tail.msh')
    # rootNode.addObject('MeshSTLLoader', name='segment0VisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/segment0.stl')
    # rootNode.addObject('MeshSTLLoader', name='segment1VisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/segment1.stl')
    # rootNode.addObject('MeshSTLLoader', name='segment2VisualLoader', filename=config["currentDirectory"]+'meshes/fullAssembly/segment2.stl')
    rootNode.addObject('MeshGmshLoader', name='couple0Loader', filename=config["currentDirectory"]+'meshes/fullAssembly/couple0_1.msh')
    rootNode.addObject('MeshGmshLoader', name='couple1Loader', filename=config["currentDirectory"]+'meshes/fullAssembly/couple1_2.msh')
    rootNode.addObject('MeshGmshLoader', name='couple2Loader', filename=config["currentDirectory"]+'meshes/fullAssembly/couple2_tail.msh')
    rootNode.addObject('MeshSTLLoader', name='chamber0_0Loader', filename=config["currentDirectory"]+'/meshes/fullAssembly/chamber0_0.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber0_1Loader', filename=config["currentDirectory"]+'/meshes/fullAssembly/chamber0_1.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber1_0Loader', filename=config["currentDirectory"]+'/meshes/fullAssembly/chamber1_0.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber1_1Loader', filename=config["currentDirectory"]+'/meshes/fullAssembly/chamber1_1.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber2_0Loader', filename=config["currentDirectory"]+'/meshes/fullAssembly/chamber2_0.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber2_1Loader', filename=config["currentDirectory"]+'/meshes/fullAssembly/chamber2_1.stl')

    ##################################################
    # head                                           #
    ##################################################
    head = rootNode.addChild('head')
    head.addObject('MeshTopology', src='@../headLoader', name='headTopologyContainer')
    head.addObject('MechanicalObject', name='state', template='Vec3', showObject=False, showObjectScale=1)
    head.addObject('EulerImplicit', name='odesolver')
    head.addObject('SparseLDLSolver', name='directSolver')
    head.addObject('LinearSolverConstraintCorrection', solverName='directSolver')

    head.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3, youngModulus=10000)
    head.addObject('UniformMass', totalMass=headMass)


    ##################################################
    # head/centerlineROI                         #
    ##################################################    
    centerline_head = head.addObject('BoxROI', template="Vec3d", name="centerline_roi", box= centerlineBounds, drawBoxes=True)


    # ################################################## TODO
    # # head/constraints                               #
    # ##################################################
    # segment0_linearConstraint= segment0.addObject('FixedConstraint', name='fixedConstraint', indices='2905 6')
    head_planarConstraint= head.addObject('PartialFixedConstraint', name='planarConstraint', indices='7 3',fixedDirections='1 0 1')



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
    segment0_couple0_attachmentROI = segment0.addObject('BoxROI', template="Vec3d", name="segment0_couple0_attachmentROI", box= segment0_couple0_attachmentBounds, drawBoxes=False)

    ##################################################
    # segment0/constraints                      #
    ##################################################
    segment0_linearConstraint= segment0.addObject('FixedConstraint', name='fixedConstraint', indices='2905 6')
    segment0_planarConstraint= segment0.addObject('PartialFixedConstraint', name='planarConstraint', indices='2907 2901 2899',fixedDirections='1 0 0')

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
    # segment2/constraints                      #
    ##################################################
    segment2_planarConstraint= segment2.addObject('PartialFixedConstraint', name='planarConstraint', indices='2905 2907 2901 2899',fixedDirections='1 0 1')


    ##################################################
    # tail                                           #
    ##################################################
    tail = rootNode.addChild('tail')
    tail.addObject('MeshTopology', src='@../tailLoader', name='tailTopologyContainer')
    tail.addObject('MechanicalObject', name='state', template='Vec3', showObject=False, showObjectScale=1)
    tail.addObject('EulerImplicit', name='odesolver')
    tail.addObject('SparseLDLSolver', name='directSolver')
    tail.addObject('LinearSolverConstraintCorrection', solverName='directSolver')
    tail.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                        youngModulus=1000)
    tail.addObject('UniformMass', totalMass=tailMass)

    ##################################################
    # tail/centerlineROI                         #
    ##################################################  
    centerlineTail = tail.addObject('BoxROI', template="Vec3d", name="centerline_roi", box= centerlineBounds,drawBoxes=True)



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
    # couple2                                        #
    ##################################################
    couple2 = rootNode.addChild('couple2')
    couple2.addObject('MeshTopology', src='@../couple2Loader', name='couple2TopologyContainer')
    couple2.addObject('MechanicalObject', name='state', template='Vec3', showObject=False, showObjectScale=1)

    couple2.addObject('EulerImplicit', name='odesolver')
    couple2.addObject('SparseLDLSolver', name='directSolver')
    couple2.addObject('LinearSolverConstraintCorrection', solverName='directSolver')
    couple2.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                        youngModulus=1000000)
    
    couple2.addObject('UniformMass', totalMass=coupleMass)



    ##################################################
    # attach head to segment0                        #
    ##################################################
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@head', object2='@segment0', first_point='7', second_point='27')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@head', object2='@segment0', first_point='3', second_point='24')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@head', object2='@segment0', first_point='5', second_point='25')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@head', object2='@segment0', first_point='2', second_point='26')    

    ##################################################
    # attach segment0 to couple0                     #
    ##################################################
    # rootNode.addObject('AttachConstraint', object1="@segment0", object2="@couple0", indices1="2887 2886 2885 2888", indices2="18 19 20 17", constraintFactor="1 1 1 1")
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@couple0', first_point='21', second_point='9')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@couple0', first_point='23', second_point='10')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@couple0', first_point='20', second_point='11')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@couple0', first_point='22', second_point='8')    

    ##################################################
    # attach segment1 to couple0                     #
    ##################################################
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple0', object2='@segment1', first_point='2', second_point='26')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple0', object2='@segment1', first_point='1', second_point='25')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple0', object2='@segment1', first_point='0', second_point='27')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple0', object2='@segment1', first_point='3', second_point='24')    
    
    
    
    ##################################################
    # attach segment1 to couple1                     #
    ################################################## 
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@couple1', first_point='21', second_point='9')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@couple1', first_point='23', second_point='10')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@couple1', first_point='20', second_point='11')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@couple1', first_point='22', second_point='8')    

    ##################################################
    # attach segment2 to couple1                     #
    ##################################################
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple1', object2='@segment2', first_point='2', second_point='26')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple1', object2='@segment2', first_point='1', second_point='25')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple1', object2='@segment2', first_point='0', second_point='27')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple1', object2='@segment2', first_point='3', second_point='24')    

    ##################################################
    # attach segment2 to couple2                     #
    ##################################################
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment2', object2='@couple2', first_point='21', second_point='9')
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment2', object2='@couple2', first_point='23', second_point='10')
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment2', object2='@couple2', first_point='20', second_point='11')
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment2', object2='@couple2', first_point='22', second_point='8')

    ##################################################
    # attach tail to couple2                         #
    ##################################################
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple2', object2='@tail', first_point='0', second_point='2')
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple2', object2='@tail', first_point='3', second_point='29')
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple2', object2='@tail', first_point='2', second_point='17')
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@couple2', object2='@tail', first_point='1', second_point='44')

    if recordCenterline: 
        rootNode.addObject(centerlineStateExporterMulti(filetype=0, name='centerlineExporter', segments = [head, segment0, segment1,segment2, tail]))
    if recordFullState: 
        rootNode.addObject(fullStateExporterMulti(filetype=0, name='stateExporter', segments = [head, segment0, segment1,segment2, tail]))
    if recordPressureInputs: 
        rootNode.addObject(pressureInputRecorder(name='inputExporter', segments = [segment0, segment1, segment2]))




def main():
    # Make sure to load all SOFA libraries and plugins
    SofaRuntime.importPlugin("SofaBaseMechanics")
    SofaRuntime.importPlugin('SofaOpenglVisual')
    for i in range(0,numEpisodes):

        # Generate the root node
        root = Sofa.Core.Node("root")

        # Call the above function to create the scene graph
        createScene(root,policySeed=i)

        # Once defined, initialization of the scene graph
        Sofa.Simulation.init(root)

        if not USE_GUI:
            for iteration in range(numSteps):
                Sofa.Simulation.animate(root, root.dt.value)
                print("Episode " + i.__str__() + " timeStep: " + iteration.__str__())
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