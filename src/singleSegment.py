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

    segment = rootNode.addChild('segment0')
    segment.addObject(Exporter(filetype=0, name='exporter'))
    segment.addObject('EulerImplicit', name='odesolver')
    segment.addObject('SparseLDLSolver', name='directSolver')

    ##################################################
    # segment                                        #
    ##################################################

    segment.addObject('MeshGmshLoader', name='loader', filename=config["currentDirectory"]+'meshes/singleSegment/Module_body3.msh')
    segment.addObject('MeshTopology', src='@loader', name='container')
    segment.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=False, showObjectScale=1)
    segment.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=1000)
    segment.addObject('UniformMass', totalMass=0.0008)

    ##################################################
    # segment/visual                                 #
    ##################################################

    segmentVisual = segment.addChild("VisualModel")
    segmentVisual.loader = segmentVisual.addObject('MeshSTLLoader', name='segmentVisualLoader', filename=config["currentDirectory"]+'/meshes/singleSegment/Module_body_visual.stl')
    segmentVisual.addObject('OglModel', name='visualModel', src='@segmentVisualLoader', color=[0.5,0.5,0.5, .25], updateNormals=False)
    segmentVisual.addObject('BarycentricMapping')
    
    ##################################################
    # segment/stiffLayer                             #
    ##################################################
    segment.addObject('BoxROI', name='boxROISubTopo', box=[0, -80, -1, 200, 80, 1], drawBoxes=False, strict=False) # Define box in which the material will be different
    segment.addObject('BoxROI', name='boxROI', box=[0, -80, -50, 10, 80, 50])
    segment.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12, angularStiffness=1e12) # spring-like boundary conditions
    segment.addObject('LinearSolverConstraintCorrection', linearSolver='@directSolver',ODESolver='@odesolver')
    # set stiff layer
    modelSubTopo = segment.addChild('modelSubTopo')
    
    # Set mesh elements of stiff layer
    modelSubTopo.addObject('MeshTopology', position='@loader.position', tetrahedra='@boxROISubTopo.tetrahedraInROI',
                           name='container')
    # Set material properties of stiff layer
    modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                           youngModulus=15000)
    # modelSubTopo.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=True, showObjectScale=1)

    # # Add mechanical object for tracking centerline state 

    ##################################################
    # segment/centerline                             #
    ################################################## 
    centerline = segment.addObject('BoxROI', template="Vec3d", name="centerline_roi", box=[0, -80, -1, 200, 80, 1],drawBoxes=True)
    segment.addObject(centerlineStateExporter(filetype=0, name='centerlineExporter'))

    # centerlineROI = segment.addChild('centerlineROI')
    # centerlineROI.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=False, showObjectScale=1)
    # centerlineROI.addObject('MeshTopology', position='@loader.position', points='@boxROI.indices',
    #                        name='container')
    # centerlineROI.addObject(Exporter(filetype=0, name='centerlineExporter'))

    ##################################################
    # segment/cavity0                                #
    ##################################################
    cavity = segment.addChild('cavity0')
    cavity.addObject('MeshSTLLoader', name='cavityLoader', filename=config["currentDirectory"]+'/meshes/singleSegment/Module_cavity1.stl')
    cavity.addObject('MeshTopology', src='@cavityLoader', name='cavityMesh')
    cavity.addObject('MechanicalObject', name='cavity')
    cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@cavityMesh.triangles', valueType='pressure')
    cavity.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

    def policy0(state,time):
        f0=100 # Hz
        f1=f0
        phi0=0
        p0= 0.01*np.max([np.sin(2*np.pi*f0*time+phi0),0])
        print("pressure0 = "+ p0.__str__())
        return p0
    

    cavity.addObject(PressureConstraintController(dt,policy0,1,cavity,name="Cavity0Controller"))


    ##################################################
    # segment/cavity1                                #
    ##################################################
    cavity = segment.addChild('cavity1')
    cavity.addObject('MeshSTLLoader', name='cavityLoader2', filename=config["currentDirectory"]+'/meshes/singleSegment/Module_cavity2.stl')
    cavity.addObject('MeshTopology', src='@cavityLoader2', name='cavityMesh2')
    cavity.addObject('MechanicalObject', name='cavity')
    cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@cavityMesh2.triangles', valueType='pressure')
    cavity.addObject('BarycentricMapping', name='mapping2', mapForces=False, mapMasses=False)

    def policy1(state,time):
        f0=100 # Hz
        f1=f0
        phi0=np.pi
        p0= 0.01*np.max([np.sin(2*np.pi*f0*time+phi0),0])
        print("pressure0 = "+ p0.__str__())
        return p0

    cavity.addObject(PressureConstraintController(dt,policy1,1,cavity,name="Cavity1Controller"))

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