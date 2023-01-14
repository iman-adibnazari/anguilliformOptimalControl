import Sofa
import Sofa.Gui
import Sofa.Simulation
from fingerController import FingerController
from fingerController2 import FingerController2
import numpy as np
import SofaRuntime
# SofaRuntime.importPlugin("SofaComponentAll")

# to create elements like Node or objects
import Sofa.Core

# Choose in your script to activate or not the GUI
USE_GUI = True
dt=0.0001

# Setup controller to save and export data

class Exporter (Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.step_id = 0
        
    def onAnimateEndEvent(self, e):
        x  = self.getContext().tetras.position.array()
        x0 = self.getContext().tetras.rest_position.array()
        u  = x - x0
        print(u)
        
        cells = self.getContext().topology.tetrahedra.array()
        # von_mises = self.getContext().ff.vonMisesPerNode.array()
        filename = f'../data/step_{self.step_id}.npy'
        np.save(filename,x)
        # meshio.write(filename, meshio.Mesh(points=x0, cells={'tetra':cells}, point_data={'u':u, 'von_mises':von_mises}))
        print(f'Mesh exported at {filename}')
        self.step_id += 1

# Setup controller to update pressure values
class PressureController0 (Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.step_id = 0
        self.node = args[0]
        self.fingerNode = self.node.getChild('finger')
        self.pressureConstraint = self.fingerNode.cavity.getObject('SurfacePressureConstraint')
        self.pressureConstraint.value = [0]
        
    def onAnimateBeginEvent(self, e):
        t=self.step_id*dt # get current time step
        f0=100 # Hz
        f1=f0
        phi0=0
        p0= 0.01*np.max([np.sin(2*np.pi*f0*t+phi0),0])
        print("pressure0 = "+ p0.__str__())
        self.pressureConstraint.value = [p0]

        self.step_id += 1

# Setup controller to update pressure values
class PressureController1 (Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.step_id = 0
        self.node = args[0]
        self.fingerNode = self.node.getChild('finger')
        self.pressureConstraint = self.fingerNode.cavity2.getObject('SurfacePressureConstraint2')
        self.pressureConstraint.value = [0]


    def onAnimateBeginEvent(self, e):
        t=self.step_id*dt # get current time step
        f0=100 # Hz
        phi0 = np.pi
        p1= 0.01*np.max([np.sin(2*np.pi*f0*t+phi0),0])
        print("pressure1 = "+ p1.__str__())
        self.pressureConstraint.value = [p1]

        self.step_id += 1


# Setup scene
def createScene(rootNode):
    rootNode.dt=dt
    rootNode.addObject('VisualStyle', displayFlags='showVisual')# showForceFields showBehavior 
    rootNode.addObject('RequiredPlugin',
                    pluginName='SoftRobots SofaPython3 SofaLoader SofaSimpleFem SofaEngine SofaDeformable SofaImplicitOdeSolver SofaConstraint SofaSparseSolver')

    rootNode.findData('gravity').value = [0, 0, 0];
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-12, maxIterations=10000)

    finger = rootNode.addChild('finger')
    finger.addObject(Exporter(name='exporter'))
    finger.addObject('EulerImplicit', name='odesolver')
    finger.addObject('SparseLDLSolver', name='directSolver')

    finger.addObject('MeshGmshLoader', name='loader', filename='../meshes/Module_body3.msh')
    finger.addObject('MeshTopology', src='@loader', name='container')
    finger.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=False, showObjectScale=1)
    finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=1000)
    finger.addObject('UniformMass', totalMass=0.0008)
    # Define visual model
    segmentVisual = finger.addChild("VisualModel")
    segmentVisual.loader = segmentVisual.addObject('MeshSTLLoader', name='segmentVisualLoader', filename='../meshes/Module_body_visual.stl')
    segmentVisual.addObject('OglModel', name='visualModel', src='@segmentVisualLoader', color=[0.5,0.5,0.5, .25], updateNormals=False)
    segmentVisual.addObject('BarycentricMapping')
    
    
    # Define stiff layer ROI
    finger.addObject('BoxROI', name='boxROISubTopo', box=[0, -80, -1, 200, 80, 1], drawBoxes=False, strict=False) # Define box in which the material will be different
    # Set spring-like boundary conditions
    finger.addObject('BoxROI', name='boxROI', box=[0, -80, -50, 10, 80, 50])
    finger.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12, angularStiffness=1e12)
    finger.addObject('LinearSolverConstraintCorrection', solverName='directSolver')
    
    
    # set stiff layer
    modelSubTopo = finger.addChild('modelSubTopo')
    # Set mesh elements of stiff layer
    modelSubTopo.addObject('MeshTopology', position='@loader.position', tetrahedra='@boxROISubTopo.tetrahedraInROI',
                           name='container')
    # Set material properties of stiff layer
    modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                           youngModulus=15000)
    
    


    # Pneumatic actuation chamber 1
    cavity = finger.addChild('cavity')
    cavity.addObject('MeshSTLLoader', name='cavityLoader', filename='../meshes/Module_cavity1.stl')
    cavity.addObject('MeshTopology', src='@cavityLoader', name='cavityMesh')
    cavity.addObject('MechanicalObject', name='cavity')
    cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@cavityMesh.triangles', valueType='pressure')
    cavity.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

    rootNode.addObject(PressureController0(rootNode))

    # Pneumatic actuation chamber 2
    cavity = finger.addChild('cavity2')
    cavity.addObject('MeshSTLLoader', name='cavityLoader2', filename='../meshes/Module_cavity2.stl')
    cavity.addObject('MeshTopology', src='@cavityLoader2', name='cavityMesh2')
    cavity.addObject('MechanicalObject', name='cavity2')
    cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint2', template='Vec3', value=1,
                     triangles='@cavityMesh2.triangles', valueType='pressure')
    cavity.addObject('BarycentricMapping', name='mapping2', mapForces=False, mapMasses=False)

    rootNode.addObject(PressureController1(rootNode))


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