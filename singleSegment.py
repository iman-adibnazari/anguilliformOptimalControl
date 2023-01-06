import Sofa
from fingerController import FingerController
from fingerController2 import FingerController2

def createScene(rootNode):
    rootNode.addObject('VisualStyle', displayFlags='showVisual')# showForceFields showBehavior 
    rootNode.addObject('RequiredPlugin',
                    pluginName='SoftRobots SofaPython3 SofaLoader SofaSimpleFem SofaEngine SofaDeformable SofaImplicitOdeSolver SofaConstraint SofaSparseSolver')

    rootNode.findData('gravity').value = [0, 0, 0];
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-12, maxIterations=10000)


    finger = rootNode.addChild('finger')
    finger.addObject('EulerImplicit', name='odesolver')
    finger.addObject('SparseLDLSolver', name='directSolver')

    finger.addObject('MeshGmshLoader', name='loader', filename='Module_body3.msh')
    finger.addObject('MeshTopology', src='@loader', name='container')
    finger.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=False, showObjectScale=1)
    finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=1000)
    finger.addObject('UniformMass', totalMass=0.0008)
    # Define visual model
    segmentVisual = finger.addChild("VisualModel")
    segmentVisual.loader = segmentVisual.addObject('MeshSTLLoader', name='segmentVisualLoader', filename='Module_body_visual.stl')
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
    cavity.addObject('MeshSTLLoader', name='cavityLoader', filename='Module_cavity1.stl')
    cavity.addObject('MeshTopology', src='@cavityLoader', name='cavityMesh')
    cavity.addObject('MechanicalObject', name='cavity')
    cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@cavityMesh.triangles', valueType='pressure')
    cavity.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

    rootNode.addObject(FingerController(rootNode))

    # Pneumatic actuation chamber 2
    cavity = finger.addChild('cavity2')
    cavity.addObject('MeshSTLLoader', name='cavityLoader2', filename='Module_cavity2.stl')
    cavity.addObject('MeshTopology', src='@cavityLoader2', name='cavityMesh2')
    cavity.addObject('MechanicalObject', name='cavity2')
    cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint2', template='Vec3', value=1,
                     triangles='@cavityMesh2.triangles', valueType='pressure')
    cavity.addObject('BarycentricMapping', name='mapping2', mapForces=False, mapMasses=False)

    rootNode.addObject(FingerController2(rootNode))