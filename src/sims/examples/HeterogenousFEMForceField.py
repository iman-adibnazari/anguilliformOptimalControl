import Sofa
import Sofa.Gui
import Sofa.Simulation
import SofaRuntime
import numpy as np
import Sofa.Core
from dotenv import dotenv_values

config = dotenv_values(".env")

# Choose in your script to activate or not the GUI
USE_GUI = True
dt=0.04

def createScene(rootNode):
    ##################################################
    # Setup scene                                    #
    ##################################################
    rootNode.dt=dt
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels showCollisionModels showInteractionForceFields showBehaviorModels')# showForceFields showBehavior')#   
    rootNode.addObject('RequiredPlugin',
                    pluginName=['Sofa.Component.Mass',
                    'Sofa.Component.Mapping',
                    'Sofa.Component.StateContainer',
                    'Sofa.Component.Topology.Container.Dynamic',
                    'Sofa.Component.Topology.Container.Grid',
                    'Sofa.GL.Component.Rendering3D',
                    'Sofa.GL.Component.Shader',
                    'Sofa.Component.Diffusion',
                    'Sofa.Component.SolidMechanics.FEM.Elastic',
                    'Sofa.Component.IO.Mesh',
                    'Sofa.Component.Engine.Select',
                    'Sofa.Component.Engine.Transform',
                    'Sofa.Component.Mapping.MappedMatrix', 
                    'Sofa.Component.Mapping.Linear', 
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
                    'Sofa.Component.LinearSolver.Iterative',
                    'SofaCUDA',
                    'Sofa.Component.MechanicalLoad',
                    'CSparseSolvers',
                    'Sofa.Component.Constraint.Projective',
                    'Sofa.Component.Visual'
                    ])

    rootNode.findData('gravity').value = [0, -9.81, 0]
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=0.0002, maxIterations=500)

    HexaTop = rootNode.addChild('Topology')
    HexaTop.addObject('RegularGridTopology', name='HexaTop', n=[9,9,18], min=[-0.05,-0.05,0.210], max=[0.05,0.05,0.295])
    HexaTop.addObject('TetrahedronSetTopologyContainer', name='Container', position="@Hexatop.position")
    HexaTop.addObject('HexahedronSetTopologyModifier', name='Modifier')
    HexaTop.addObject('Hexa2TetraTopologicalMapping', input='@HexaTop', output='@Container',swapping="false")

    FEM = rootNode.addChild('FEM')
    FEM.addObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
    FEM.addObject('SparseLDLSolver', name='precond',template='CompressedRowSparseMatrixMat3x3')
    FEM.addObject('ShewchukPCGLinearSolver', preconditioners="@precond", update_step=15)

    FEM.addObject('TetrahedronSetTopologyContainer', name='Container', position="@../Topology/HexaTop.position", tetrahedra="@../Topology/Container.tetrahedra")
    FEM.addObject('TetrahedronSetTopologyModifier', name='Modifier')
    FEM.addObject('MechanicalObject', name='mstate', template="Vec3", showIndices="0")
    # FEM.addObject('BoxConstraint', name='support', box="-0.06 -0.06 0.29 0.06 0.06 0.295",  drawBoxes="true")

    FEM.addObject('BoxROI', name="Gel1", box="-0.05 -0.05 0.210 0.05 0.05 0.225", drawBoxes="true")
    FEM.addObject('BoxROI', name="Gel2", box="-0.05 -0.05 0.2245 0.05 0.05 0.245", drawBoxes="true")
    FEM.addObject('BoxROI', name="Gel3", box="-0.05 -0.05 0.245 0.05 0.05 0.260", drawBoxes="true")
    FEM.addObject('BoxROI', name="Gel4", box="-0.05 -0.05 0.260 0.05 0.05 0.295", drawBoxes="true")




def main():
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