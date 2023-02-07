import Sofa
import Sofa.Gui
import Sofa.Simulation
import numpy as np
import SofaRuntime
from fullStateRecorder import Exporter
from pressureConstraintController import PressureConstraintController
from centerlineStateRecorder import centerlineStateExporter
# to create elements like Node or objects
import Sofa.Core
from dotenv import dotenv_values  

# Import policy 
from randomPolicy import randomPolicy

config = dotenv_values(".env")

# Choose in your script to activate or not the GUI
USE_GUI = True
dt=0.0001
attachPumps = 1


def createScene(rootNode):
    ##################################################
    # Setup scene                                    #
    ##################################################
    rootNode.dt=dt
    rootNode.addObject('VisualStyle', displayFlags='showVisual')# showForceFields showBehavior 
    rootNode.addObject('RequiredPlugin',
                    pluginName='SoftRobots SofaPython3 SofaLoader SofaSimpleFem SofaEngine SofaDeformable SofaImplicitOdeSolver SofaConstraint SofaSparseSolver')

    rootNode.findData('gravity').value = [0, 0, 0]
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-12, maxIterations=10000)



    ##################################################
    # Load in meshes                                 #
    ##################################################
    rootNode.addObject('MeshGmshLoader', name='segment0Loader', filename=config["currentDirectory"]+'meshes/twoSegments/segment0.msh')
    rootNode.addObject('MeshGmshLoader', name='segment1Loader', filename=config["currentDirectory"]+'meshes/twoSegments/segment1.msh')
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
    segment0.addObject('EulerImplicit', name='odesolver')
    segment0.addObject('SparseLDLSolver', name='directSolver')
    segment0.addObject('MeshTopology', src='@../segment0Loader', name='segment0TopologyContainer')
    segment0.addObject('MechanicalObject', name='segment0State', template='Vec3', showObject=False, showObjectScale=1)
    segment0.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=1000)
    segment0.addObject('UniformMass', totalMass=0.0008)


    
    ##################################################
    # segment0/visual                                #
    ##################################################
    segment0Visual = segment0.addChild("VisualModel")
    segment0Visual.addObject('OglModel', name='visualModel', src='@../../segment0Loader', color=[0.5,0.5,0.5, 1], updateNormals=False)
    segment0Visual.addObject('BarycentricMapping')

    ##################################################
    # segment0/chamber0_0                            #
    ################################################## 
    chamber0_0 = segment0.addChild('chamber0_0')
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
    

    # chamber0_0.addObject(PressureConstraintController(dt,policy0,1,chamber0_0,name="chamber0_0Controller"))

    ##################################################
    # segment0/chamber0_1                            #
    ##################################################    
    chamber0_1 = segment0.addChild('chamber0_1')
    chamber0_1.addObject('MeshTopology', src='@../../chamber0_1Loader', name='chamber0_1Mesh')
    chamber0_1.addObject('MechanicalObject', name='chamber0_1')
    chamber0_1.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@chamber0_1Mesh.triangles', valueType='pressure')
    chamber0_1.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)


    ##################################################
    # segment1                                       #
    ##################################################
    segment1 = rootNode.addChild('segment1')
    segment1.addObject('EulerImplicit', name='odesolver')
    segment1.addObject('SparseLDLSolver', name='directSolver')
    segment1.addObject('MeshTopology', src='@../segment1Loader', name='segment1TopologyContainer')
    segment1.addObject('MechanicalObject', name='segment1State', template='Vec3', showObject=False, showObjectScale=1)
    segment1.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=1000)
    segment1.addObject('UniformMass', totalMass=0.0008)
    ##################################################
    # segment1/visual                                #
    ##################################################
    segment1Visual = segment0.addChild("VisualModel")
    segment1Visual.addObject('OglModel', name='visualModel', src='@../../segment1Loader', color=[0.5,0.5,0.5, 1], updateNormals=False)
    segment1Visual.addObject('BarycentricMapping')
    ##################################################
    # segment1/chamber1_0                            #
    ##################################################
    chamber1_0 = segment1.addChild('chamber1_0')
    chamber1_0.addObject('MeshTopology', src='@../../chamber1_0Loader', name='chamber1_0Mesh')
    chamber1_0.addObject('MechanicalObject', name='chamber1_0')
    chamber1_0.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@chamber1_0Mesh.triangles', valueType='pressure')
    chamber1_0.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

    ##################################################
    # segment1/chamber1_1                            #
    ##################################################
    chamber1_1 = segment1.addChild('chamber1_1')
    chamber1_1.addObject('MeshTopology', src='@../../chamber1_1Loader', name='chamber1_1Mesh')
    chamber1_1.addObject('MechanicalObject', name='chamber1_1')
    chamber1_1.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@chamber1_1Mesh.triangles', valueType='pressure')
    chamber1_1.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)


    ##################################################
    # couple0                                        #
    ##################################################
    couple0 = rootNode.addChild('couple0')
    couple0.addObject('EulerImplicit', name='odesolver')
    couple0.addObject('SparseLDLSolver', name='directSolver')
    couple0.addObject('MeshTopology', src='@../couple0Loader', name='couple0TopologyContainer')
    couple0.addObject('MechanicalObject', name='couple0State', template='Vec3', showObject=False, showObjectScale=1)
    couple0.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=1000)
    couple0.addObject('UniformMass', totalMass=0.0008)


    ##################################################
    # couple0/visual                                 #
    ##################################################
    couple0Visual = couple0.addChild("VisualModel")
    couple0Visual.addObject('OglModel', name='visualModel', src='@../../couple0Loader', color=[0.7,0.7,1, 0.1], updateNormals=False)
    couple0Visual.addObject('BarycentricMapping')

    ##################################################
    # pump0                                          #
    ##################################################
    pump0 = rootNode.addChild('pump0')
    pump0.addObject('EulerImplicit', name='odesolver')
    pump0.addObject('SparseLDLSolver', name='directSolver')
    pump0.addObject('MeshTopology', src='@../pump0Loader', name='pump0TopologyContainer')
    pump0.addObject('MechanicalObject', name='pump0State', template='Vec3', showObject=False, showObjectScale=1)
    pump0.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=1000)
    pump0.addObject('UniformMass', totalMass=0.0008)

    ##################################################
    # pump0/visual                                   #
    ##################################################
    pump0Visual = pump0.addChild("VisualModel")
    pump0Visual.addObject('OglModel', name='visualModel', src='@../../pump0Loader', color=[0.7,1,0.7, 0.1], updateNormals=False)
    pump0Visual.addObject('BarycentricMapping')
    ##################################################
    # pump1                                          #
    ##################################################
    pump1 = rootNode.addChild('pump1')
    pump1.addObject('EulerImplicit', name='odesolver')
    pump1.addObject('SparseLDLSolver', name='directSolver')
    pump1.addObject('MeshTopology', src='@../pump1Loader', name='pump1TopologyContainer')
    pump1.addObject('MechanicalObject', name='pump1State', template='Vec3', showObject=False, showObjectScale=1)
    pump1.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=1000)
    pump1.addObject('UniformMass', totalMass=0.0008)

    ##################################################
    # pump1/visual                                   #
    ##################################################
    pump1Visual = pump1.addChild("VisualModel")
    pump1Visual.addObject('OglModel', name='visualModel', src='@../../pump1Loader', color=[0.7,1,0.7, 0.1], updateNormals=False)
    pump1Visual.addObject('BarycentricMapping')


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
        for iteration in range(10):
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