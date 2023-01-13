import Sofa
import Sofa.Core
import Sofa.Simulation
import SofaRuntime
import Sofa.Gui
SofaRuntime.importPlugin('SofaOpenglVisual')





# Function creating the scene and describing the graph (with nodes and components) before it is initialized
def createScene(root):
        return root

#Main function ONLY called if this script is called from a python environment
def main():

        # Make sure to load all SOFA libraries and plugins
        SofaRuntime.importPlugin("SofaBaseMechanics")

        # Generate the root node
        root = Sofa.Core.Node("root")

        # Call the above function to create the scene graph
        createScene(root)

        # Once defined, initialization of the scene graph
        Sofa.Simulation.init(root)

        # Run the simulation for 10 steps
        for iteration in range(10):
                Sofa.Simulation.animate(root, root.dt.value)
                # print(root)

# Function used only if this script is called from a python environment, triggers the main()
if __name__ == '__main__':
    main()