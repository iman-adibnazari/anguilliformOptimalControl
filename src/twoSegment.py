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
numSteps = 500
dt=0.0005
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

sinusoidalPolicy_c00 = sinusoidalPolicy(dt=dt, phase = 180, amplitude = 0.025, frequency = 8) 
sinusoidalPolicy_c01 = sinusoidalPolicy(dt=dt, phase = 0, amplitude = 0.025, frequency = 8)
sinusoidalPolicy_c10 = sinusoidalPolicy(dt=dt, phase = 0, amplitude = 0.025, frequency = 8)
sinusoidalPolicy_c11 = sinusoidalPolicy(dt=dt, phase = 180, amplitude = 0.025, frequency = 8)



def createScene(rootNode):
    ##################################################
    # Setup scene                                    #
    ##################################################
    rootNode.dt=dt
    rootNode.addObject('VisualStyle', displayFlags='showVisual')# showForceFields showBehavior 
    rootNode.addObject('RequiredPlugin',
                    pluginName='SoftRobots SofaPython3 SofaLoader SofaSimpleFem SofaEngine SofaDeformable SofaImplicitOdeSolver SofaConstraint SofaSparseSolver')

    rootNode.findData('gravity').value = [0, -9.81, 0]
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-12, maxIterations=10000)



    ##################################################
    # Load in meshes                                 #
    ##################################################
    rootNode.addObject('MeshGmshLoader', name='segment0Loader', filename=config["currentDirectory"]+'meshes/twoSegments/segment0.msh')
    rootNode.addObject('MeshGmshLoader', name='segment1Loader', filename=config["currentDirectory"]+'meshes/twoSegments/segment1.msh')
    rootNode.addObject('MeshSTLLoader', name='segment0VisualLoader', filename=config["currentDirectory"]+'meshes/twoSegments/segment0.stl')
    rootNode.addObject('MeshSTLLoader', name='segment1VisualLoader', filename=config["currentDirectory"]+'meshes/twoSegments/segment1.stl')
    rootNode.addObject('MeshGmshLoader', name='pump0Loader', filename=config["currentDirectory"]+'meshes/twoSegments/pump0.msh')
    rootNode.addObject('MeshGmshLoader', name='pump1Loader', filename=config["currentDirectory"]+'meshes/twoSegments/pump1.msh')
    rootNode.addObject('MeshGmshLoader', name='couple0Loader', filename=config["currentDirectory"]+'meshes/twoSegments/couple0.msh')
    rootNode.addObject('MeshSTLLoader', name='chamber0_0Loader', filename=config["currentDirectory"]+'/meshes/twoSegments/chamber0_0.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber0_1Loader', filename=config["currentDirectory"]+'/meshes/twoSegments/chamber0_1.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber1_0Loader', filename=config["currentDirectory"]+'/meshes/twoSegments/chamber1_0.stl')
    rootNode.addObject('MeshSTLLoader', name='chamber1_1Loader', filename=config["currentDirectory"]+'/meshes/twoSegments/chamber1_1.stl')

    # TODO: Maybe get meshes for visuals or use mapping


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
    
    ##################################################
    # segment0/visual                                #
    ##################################################
    segment0Visual = segment0.addChild("VisualModel")
    
    segment0Visual.addObject('OglModel', name='visualModel', src='@../../segment0VisualLoader', color=[0.5,0.5,0.5, 0.5], updateNormals=False)
    segment0Visual.addObject('BarycentricMapping')

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

    def policy0(state,time):
        f0=100 # Hz
        f1=f0
        phi0=0
        p0= 0.01*np.max([np.sin(2*np.pi*f0*time+phi0),0])
        print("pressure0 = "+ p0.__str__())
        return p0
    

    chamber0_0.addObject(PressureConstraintController(dt,sinusoidalPolicy_c00,savePressureInputs,chamber0_0,name="chamber0_0Controller"))

    ##################################################
    # segment0/chamber1                            #
    ##################################################    
    chamber0_1 = segment0.addChild('chamber1')
    chamber0_1.addObject('MeshTopology', src='@../../chamber0_1Loader', name='chamber0_1Mesh')
    chamber0_1.addObject('MechanicalObject', name='chamber0_1')
    chamber0_1.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@chamber0_1Mesh.triangles', valueType='pressure')
    chamber0_1.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

    def policy1(state,time):
        f0=100 # Hz
        f1=f0
        phi0=np.pi
        p0= 0.01*np.max([np.sin(2*np.pi*f0*time+phi0),0])
        print("pressure0 = "+ p0.__str__())
        return p0
    

    chamber0_1.addObject(PressureConstraintController(dt,sinusoidalPolicy_c01,savePressureInputs,chamber0_1,name="chamber0_1Controller"))

    ##################################################
    # segment0/centerlineROI                         #
    ##################################################    
    centerline0 = segment0.addObject('BoxROI', template="Vec3d", name="centerline_roi", box=[-132, -80, 14.88, 400, 80, 16.89],drawBoxes=False)

    ##################################################
    # attach reference frame to segment 0            #
    ##################################################

    refFrameROI = segment0.addObject('BoxROI', template="Rigid3d", name="refFrame", box=[-132, 70, 12.88, -122, 90, 18.89],drawBoxes=True)
    
    ##################################################
    # segment0/planarConstraint                      #
    ##################################################
    planarConstraint = segment0.addObject('PartialFixedConstraint', name='planarConstraint', indices='499', fixedDirections='0 0 0')

    planarConstraint = segment0.addObject('PartialFixedConstraint', name='planarConstraint', indices='505 498 504', fixedDirections='1 0 1')
    
    # refFrameSubSet = rootNode.addChild('MechanicalObject', name='refFrameSubSet', src='@../segment0.state', subset='@../segment0.refFrame.indices')
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
    ##################################################
    # segment1/visual                                #
    ##################################################
    segment1Visual = segment1.addChild("VisualModel")
    segment1Visual.addObject('OglModel', name='visualModel', src='@../../segment1VisualLoader', color=[0.5,0.5,0.5, 0.5], updateNormals=False)
    segment1Visual.addObject('BarycentricMapping')


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
    
    def policy1(state,time):
        f0=100 # Hz
        f1=f0
        phi0=np.pi
        p0= 0.01*np.max([np.sin(2*np.pi*f0*time+phi0),0])
        print("pressure0 = "+ p0.__str__())
        return p0

    chamber0_1.addObject(PressureConstraintController(dt,sinusoidalPolicy_c10,savePressureInputs,chamber1_0,name="chamber1_0Controller"))

    ##################################################
    # segment1/chamber1                            #
    ##################################################
    chamber1_1 = segment1.addChild('chamber1')
    chamber1_1.addObject('MeshTopology', src='@../../chamber1_1Loader', name='chamber1_1Mesh')
    chamber1_1.addObject('MechanicalObject', name='chamber1_1')
    chamber1_1.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@chamber1_1Mesh.triangles', valueType='pressure')
    chamber1_1.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

    def policy0(state,time):
        f0=100 # Hz
        f1=f0
        phi0=0
        p0= 0.01*np.max([np.sin(2*np.pi*f0*time+phi0),0])
        print("pressure0 = "+ p0.__str__())
        return p0
    

    chamber0_0.addObject(PressureConstraintController(dt,sinusoidalPolicy_c11,savePressureInputs,chamber1_1,name="chamber1_1Controller"))
    
    ##################################################
    # segment1/centerlineROI                         #
    ##################################################  
    centerline1 = segment1.addObject('BoxROI', template="Vec3d", name="centerline_roi", box=[-132, -80, 14.88, 400, 80, 16.89],drawBoxes=False)
   
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


    ##################################################
    # couple0/visual                                 #
    ##################################################
    couple0Visual = couple0.addChild("VisualModel")

    couple0Visual.addObject('OglModel', name='visualModel', src='@../../couple0Loader', color=[0.7,0.7,1, 0.5], updateNormals=False)
    couple0Visual.addObject('BarycentricMapping')


    ##################################################
    # attach segment0 to couple0                     #
    ################################################## 
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@couple0', first_point='1598', second_point='112')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@couple0', first_point='511', second_point='19')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@couple0', first_point='510', second_point='17')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@couple0', first_point='513', second_point='16')    

    ##################################################
    # attach segment1 to couple0                     #
    ##################################################
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@couple0', first_point='526', second_point='22')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@couple0', first_point='528', second_point='23')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@couple0', first_point='529', second_point='21')    
    rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@couple0', first_point='527', second_point='20')    


    if recordCenterline: 
        rootNode.addObject(centerlineStateExporterMulti(filetype=0, name='centerlineExporter', segments = [segment0, segment1]))
    if recordFullState: 
        rootNode.addObject(fullStateExporterMulti(filetype=0, name='stateExporter', segments = [segment0, segment1]))
    if recordPressureInputs: 
        rootNode.addObject(pressureInputRecorder(name='inputExporter', segments = [segment0, segment1]))



    if attachPumps: 

        ##################################################
        # pump0                                          #
        ##################################################
        pump0 = rootNode.addChild('pump0')
        pump0.addObject('MeshTopology', src='@../pump0Loader', name='pump0TopologyContainer')
        pump0.addObject('MechanicalObject', name='state', template='Vec3', showObject=False, showObjectScale=1)
        pump0.addObject('EulerImplicit', name='odesolver')
        pump0.addObject('SparseLDLSolver', name='directSolver')
        pump0.addObject('LinearSolverConstraintCorrection', solverName='directSolver')
        pump0.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                        youngModulus=100000)
        pump0.addObject('UniformMass', totalMass=0.00008)

        ##################################################
        # pump0/visual                                   #
        ##################################################
        pump0Visual = pump0.addChild("VisualModel")
        pump0Visual.addObject('OglModel', name='visualModel', src='@../../pump0Loader', color=[0.7,1,0.7, 0.5], updateNormals=False)
        pump0Visual.addObject('BarycentricMapping')
        ##################################################
        # pump1                                          #
        ##################################################
        pump1 = rootNode.addChild('pump1')
        pump1.addObject('MeshTopology', src='@../pump1Loader', name='pump1TopologyContainer')
        pump1.addObject('MechanicalObject', name='state', template='Vec3', showObject=False, showObjectScale=1)
        pump1.addObject('EulerImplicit', name='odesolver')
        pump1.addObject('SparseLDLSolver', name='directSolver')
        pump1.addObject('LinearSolverConstraintCorrection', solverName='directSolver')
        pump1.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                        youngModulus=100000)
        pump1.addObject('UniformMass', totalMass=0.00008)

        ##################################################
        # pump1/visual                                   #
        ##################################################
        pump1Visual = pump1.addChild("VisualModel")
        pump1Visual.addObject('OglModel', name='visualModel', src='@../../pump1Loader', color=[0.7,1,0.7, 0.5], updateNormals=False)
        pump1Visual.addObject('BarycentricMapping')






        ##################################################
        # attach segment0 to pump0                       #
        ##################################################
        rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@pump0', first_point='1638', second_point='22')    
        rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@pump0', first_point='618', second_point='42')    
        rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment0', object2='@pump0', first_point='3059', second_point='187')    



        ##################################################
        # attach segment1 to pump1                       #
        ##################################################
        rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@pump1', first_point='1870', second_point='4')    
        rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@pump1', first_point='625', second_point='9')    
        rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@pump1', first_point='624', second_point='32')    
        rootNode.addObject('BilateralInteractionConstraint', template='Vec3d', object1='@segment1', object2='@pump1', first_point='3865', second_point='174')    



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