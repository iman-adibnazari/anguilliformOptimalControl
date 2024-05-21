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
    rootNode.addObject('VisualStyle', displayFlags='showVisual')# showForceFields showBehavior')#   
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
                    'CSparseSolvers'
                    ])

    rootNode.findData('gravity').value = [0, -9.81, 0]
    rootNode.addObject('DefaultAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-12, maxIterations=10000)

    ##################################################
    # Floor                                          #
    ##################################################
    floor = rootNode.addChild('Floor')
        # Add mechanicalObject to the floor
    floor.addObject('MechanicalObject', template='CudaVec3f', name='DOFs')
    # Add regularGridTopology to the floor
    floor.addObject('RegularGridTopology', n=[2, 1, 2], min=[20, -3.05, -20], max=[-20, -3.05, 20])
    # Add visualModel to the floor
    floorVisual = floor.addChild("VisualModel")
    floorVisual.addObject('OglModel', color='red',name='Visual')
    floorVisual.addObject('IdentityMapping', input='@..', output='@Visual')

    ##################################################
    # SquareCloth                                    #
    ##################################################
    squareCloth = rootNode.addChild('SquareCloth')
    squareCloth.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.05, rayleighMass=0.05)
    squareCloth.addObject('CGLinearSolver', name='linearSolver', iterations=10, threshold=0.000001,tolerance=1e-5)
    # squareCloth.addObject('SparseLDLSolver', name='linearSolver')
    squareCloth.addObject('MechanicalObject', template='CudaVec3f', name='DOFs')
    squareCloth.addObject('UniformMass', totalMass=100)
    squareCloth.addObject('RegularGridTopology', n=[100, 1, 100], min=[12, 7, -12], max=[-12, 7, 12])
    squareCloth.addObject('BoxROI', box=[[-12, 7, 12, -10, 7, 12],[10, 7, 12, 12, 7, 12]], drawBoxes=True, name='BoxROI')
    squareCloth.addObject('FixedConstraint', indices = '@BoxROI.indices')
    # squareCloth.addObject('BoxConstraint', box=[10, 7, 12, 12, 7, 12], drawBoxes=True)
    squareCloth.addObject('MeshSpringForceField', stiffness=1000, damping=0, name = 'Springs')
    squareCloth.addObject('QuadBendingSprings', stiffness=1000, damping=0, name = 'Springs')
    squareCloth.addObject('SphereForceField', stiffness=1000, damping=1, center=[0, 1, 3], radius=4)
    squareCloth.addObject('PlaneForceField', stiffness=1000, damping=20, normal=[0, 1, 0], d=-3)

    # Add visualModel to the squareCloth
    squareClothVisual = squareCloth.addChild("VisualModel")
    squareClothVisual.addObject('OglModel', color='green',name='Visual')
    squareClothVisual.addObject('IdentityMapping', input='@..', output='@Visual')

    
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