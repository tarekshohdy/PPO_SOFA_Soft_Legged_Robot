# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
from softrobots.actuators import PullingCable
from stlib3.physics.collision import CollisionMesh
from splib3.loaders import loadPointListFromFile
import numpy as np

USE_GUI = False

root = Sofa.Core.Node("root")

def init_simulation(youngmodulus: float, poison: float) -> None:
    '''
    Initialize the simulation with the given parameters.

    Parameters:
        youngmodulus (float): Young's modulus of the material
        poison (float): Poisson's ratio of the material

    Returns:
        None
    '''

    createScene(root, poissonRatio=poison, youngModulus=youngmodulus) 
    
    Sofa.Simulation.init(root)
    

def run_simulation(cable_inputs: np.array) -> np.array:
    '''
    Run the simulation with the given parameters and return the output of the leg.

    Parameters:
        youngmodulus (float): Young's modulus of the material
        poison (float): Poisson's ratio of the material
        cable_inputs (np.array): Array with the three cable lengths

    Returns:
        np.array: Array with the x, y, z coordinates of the leg
    '''

    root.Leg.leg.PullingCable1.CableConstraint.value = [cable_inputs[0]]
    root.Leg.leg.PullingCable2.CableConstraint.value = [cable_inputs[1]]
    root.Leg.leg.PullingCable3.CableConstraint.value = [cable_inputs[2]]
    Sofa.Simulation.animate(root, root.dt.value)
    leg_output = np.array(root.Leg.leg.CollisionMesh.MechanicalObject.position.value[-34])

    return leg_output

def unload_simulation() -> None:
    '''
    Unload the simulation.

    Parameters:
        None

    Returns:
        None
    '''

    Sofa.Simulation.unload(root)

class TimerController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        # This is needed to avoid a conflict with the timer of runSofa
        self.use_sofa_profiler_timer = False

    def onAnimateBeginEvent(self, event):
        pass
        # print("Position")
        # print(rootNode.Finger.leg.sphere.mstate.position.value)

    def onAnimateEndEvent(self, event):
        pass

def Leg(parentNode=None, name="Leg",
           rotation=[90.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[30.0, -30.0, 5.0, -30.0, 30.0, -10.0], pullPointLocation=[0.0, 0.0, 0.0]
           , poissonRatio=0.3, youngModulus=18000):
    
    leg = parentNode.addChild(name)
    eobject = ElasticMaterialObject(leg,
                                    volumeMeshFileName="mesh/Solid_Cylinder_Coarse.vtk",
                                    poissonRatio=poissonRatio,
                                    youngModulus=youngModulus,
                                    totalMass=0.5,
                                    surfaceColor=[0.0, 0.8, 0.7, 1.0],
                                    surfaceMeshFileName="mesh/Solid_Cylinder.stl",
                                    rotation=rotation,
                                    translation=translation,
                                    name="leg")

    leg.addChild(eobject)

    FixedBox(eobject, atPositions=fixingBox, doVisualization=True)

    cable1 = PullingCable(eobject,
                         "PullingCable1",
                         pullPointLocation=pullPointLocation,
                         rotation=[0.0, 90.0, -90.0],
                         translation=[0.0, 0.0, 0.0],
                         cableGeometry=loadPointListFromFile("mesh/string.json"));

    cable2 = PullingCable(eobject,
                         "PullingCable2",
                         pullPointLocation=pullPointLocation,
                         rotation=[120.0, 90.0, -90.0],
                         translation=[0.0, 0.0, 0.0],
                         cableGeometry=loadPointListFromFile("mesh/string.json"));

    cable3 = PullingCable(eobject,
                         "PullingCable3",
                         pullPointLocation=pullPointLocation,
                         rotation=[-120.0, 90.0, -90],
                         translation=[0.0, 0.0, 0.0],
                         cableGeometry=loadPointListFromFile("mesh/string.json"));

    CollisionMesh(eobject, name="CollisionMesh",
                  surfaceMeshFileName="mesh/Solid_Cylinder.stl",
                  rotation=rotation, translation=translation,
                  collisionGroup=[1, 2])

    return leg


def createScene(rootNode, poissonRatio, youngModulus):
    from stlib3.scene import MainHeader, ContactHeader
    rootNode.dt = 0.01

    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Algorithm", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.AnimationLoop", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Intersection", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Lagrangian.Correction", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.IO.Mesh", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Direct", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.SolidMechanics.FEM.Elastic", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Engine.Select", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.GL.Component.Rendering3D", printLog=False)
    # rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    MainHeader(rootNode, gravity=[0.0, 0.0, -981.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, frictionCoef=0.08)
    # rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"
    rootNode.addObject( TimerController() )

    Leg(rootNode, translation=[0.0, 0.0, 0.0], poissonRatio=poissonRatio, youngModulus=youngModulus)

    return rootNode

# print(run_simulation(1.35865873e+04, 1.53066313e-01, [10.430993010570383,16.78650966142401,17.153028804342817]))

# init_simulation(21054.42434106176, 0.09326696158645303)
# print(run_simulation([0.0,14.211586324499272,0.33375650374591725]))