# -*- coding: utf-8 -*-
import soft_legged_robot_env

path = soft_legged_robot_env.__path__[0]

import Sofa.Core
import Sofa.constants.Key as Key
from stlib3.physics.deformable import ElasticMaterialObject
from softrobots.actuators import PullingCable
from stlib3.physics.collision import CollisionMesh
from splib3.loaders import loadPointListFromFile
from soft_legged_robot_env.envs.utils import *

root = Sofa.Core.Node("root")

B = matrix_B([0, 0, 44], [8.70994, -12.0554, 44], [17.6279, 0.277602, 44])

def init_simulation() -> None:
    '''
    Initialize the simulation with the given parameters.

    Parameters:
        None
    Returns:
        None
    '''
    
    createScene(root) 
    
    Sofa.Simulation.init(root)

def run_simulation(cable_inputs: np.array) -> np.array:
    '''
    Run the simulation with the given parameters and return the output of the leg.

    Parameters:
        cable_inputs (np.array): Array with the 9 cable lengths

    Returns:
        np.array: Array with the x, y, z coordinates of the leg
    '''

    root.Robot.robot.PullingCable1_1.CableConstraint.value = [cable_inputs[0]]
    root.Robot.robot.PullingCable1_2.CableConstraint.value = [cable_inputs[1]]
    root.Robot.robot.PullingCable1_3.CableConstraint.value = [cable_inputs[2]]
    root.Robot.robot.PullingCable2_1.CableConstraint.value = [cable_inputs[3]]
    root.Robot.robot.PullingCable2_2.CableConstraint.value = [cable_inputs[4]]
    root.Robot.robot.PullingCable2_3.CableConstraint.value = [cable_inputs[5]]
    root.Robot.robot.PullingCable3_1.CableConstraint.value = [cable_inputs[6]]
    root.Robot.robot.PullingCable3_2.CableConstraint.value = [cable_inputs[7]]
    root.Robot.robot.PullingCable3_3.CableConstraint.value = [cable_inputs[8]]
    Sofa.Simulation.animate(root, root.dt.value)
    point1 = root.Robot.robot.CollisionMesh.MechanicalObject.position.value[[848]][0]
    point2 = root.Robot.robot.CollisionMesh.MechanicalObject.position.value[[849]][0]
    point3 = root.Robot.robot.CollisionMesh.MechanicalObject.position.value[[850]][0]
    roll, pitch, yaw = calculate_roll_pitch_yaw(B, point1, point2, point3)
    pose = np.array([point1[0], point1[1], point1[2], roll, pitch, yaw])
    vel = np.array(root.Robot.robot.CollisionMesh.MechanicalObject.velocity.value[848])
    # print(vel)
    return pose , vel

def reset_simulation() -> None:
    '''
    Reset the simulation to its initial state.

    Parameters:
        None
        
    Returns:
        None
    '''

    Sofa.Simulation.reset(root)
    Sofa.Simulation.animateNSteps(root, 5 ,root.dt.value)
    point1 = root.Robot.robot.CollisionMesh.MechanicalObject.position.value[[848]][0]
    point2 = root.Robot.robot.CollisionMesh.MechanicalObject.position.value[[849]][0]
    point3 = root.Robot.robot.CollisionMesh.MechanicalObject.position.value[[850]][0]
    roll, pitch, yaw = calculate_roll_pitch_yaw(B, point1, point2, point3)
    pose = np.array([point1[0], point1[1], point1[2], roll, pitch, yaw])
    vel = np.array(root.Robot.robot.CollisionMesh.MechanicalObject.velocity.value[848])

    return pose, vel

def Robot(parentNode=None, name="Robot",
           rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0]):

    robot = parentNode.addChild(name)
    eobject = ElasticMaterialObject(robot,
                                    volumeMeshFileName=path + "/envs/mesh/robot_50mm.vtk",
                                    poissonRatio=0.0821706941,
                                    youngModulus=15997.6155,
                                    totalMass=2,
                                    surfaceColor=[1.0, 1.0, 1.0, 1.0],
                                    surfaceMeshFileName=path + "/envs/mesh/robot_50mm.stl",
                                    rotation=rotation,
                                    translation=translation,
                                    name="robot")

    robot.addChild(eobject)

    CollisionMesh(eobject, name="CollisionMesh",
                  surfaceMeshFileName=path + "/envs/mesh/robot_50mm.stl",
                  rotation=rotation, translation=translation,
                  collisionGroup=[1, 2])

    cable1_1 = PullingCable(eobject,
                         "PullingCable1_1",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, 0.0],
                         translation=[-57.74, 100.0, 0.0],
                         cableGeometry=loadPointListFromFile(path + "/envs/mesh/string.json"));

    cable1_2 = PullingCable(eobject,
                         "PullingCable1_2",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, 120.0],
                         translation=[-57.74, 100.0, 0.0],
                         cableGeometry=loadPointListFromFile(path + "/envs/mesh/string.json"));

    cable1_3 = PullingCable(eobject,
                         "PullingCable1_3",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, -120.0],
                         translation=[-57.74, 100.0, 0.0],
                         cableGeometry=loadPointListFromFile(path + "/envs/mesh/string.json"));

    cable2_1 = PullingCable(eobject,
                         "PullingCable2_1",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, 0.0],
                         translation=[-57.74, -100.0, 0.0],
                         cableGeometry=loadPointListFromFile(path + "/envs/mesh/string.json"));
    
    cable2_2 = PullingCable(eobject,
                         "PullingCable2_2",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, 120.0],
                         translation=[-57.74, -100.0, 0.0],
                         cableGeometry=loadPointListFromFile(path + "/envs/mesh/string.json"));
    
    cable2_3 = PullingCable(eobject,
                         "PullingCable2_3",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, -120.0],
                         translation=[-57.74, -100.0, 0.0],
                         cableGeometry=loadPointListFromFile(path + "/envs/mesh/string.json"));
    
    cable3_1 = PullingCable(eobject,
                         "PullingCable3_1",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, 0.0],
                         translation=[115.47, 0.0, 0.0],
                         cableGeometry=loadPointListFromFile(path + "/envs/mesh/string.json"));

    cable3_2 = PullingCable(eobject,
                         "PullingCable3_2",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, 120.0],
                         translation=[115.47, 0.0, 0.0],
                         cableGeometry=loadPointListFromFile(path + "/envs/mesh/string.json"));

    cable3_3 = PullingCable(eobject,
                         "PullingCable3_3",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, -120.0],
                         translation=[115.47, 0.0, 0.0],
                         cableGeometry=loadPointListFromFile(path + "/envs/mesh/string.json"));
    
    totalMass = 1.0
    volume = 1.0
    inertiaMatrix=[1., 0., 0., 0., 1., 0., 0., 0., 1.]

    floor = robot.addChild("floor")

    floor.addObject('MechanicalObject', name="mstate", template="Rigid3", translation2=[0.0, 0.0, -200.0], rotation2=[90., 0., 0.], showObjectScale=1.0)
    floor.addObject('UniformMass', name="mass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    floorCollis = floor.addChild('collision')
    floorCollis.addObject('MeshOBJLoader', name="loader", filename= "mesh/floor.obj", triangulate="true", scale3d=[50.0, 5.0, 50.0])
    floorCollis.addObject('MeshTopology', src="@loader")
    floorCollis.addObject('MechanicalObject')
    floorCollis.addObject('TriangleCollisionModel', moving=False, simulated=False)
    floorCollis.addObject('LineCollisionModel', moving=False, simulated=False)
    floorCollis.addObject('PointCollisionModel', moving=False, simulated=False)
    floorCollis.addObject('RigidMapping')

    floorVisu = floor.addChild("VisualModel")
    floorVisu.loader = floorVisu.addObject('MeshOBJLoader', name="loader", filename= "mesh/floor.obj")
    floorVisu.addObject('OglModel', name="model", src="@loader", scale3d=[50.0, 5.0, 50.0], color=[0.6, 0.1, 0.2], updateNormals=False)
    floorVisu.addObject('RigidMapping')

    return robot


def createScene(rootNode):
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
    rootNode.addObject('RequiredPlugin', name="Sofa.GL.Component.Shader")
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    MainHeader(rootNode, gravity=[0.0, 0.0, -9810.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=0.1*100, contactDistance=0.05*100, frictionCoef=5)
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"

    rootNode.addObject('LightManager', )
    rootNode.addObject('SpotLight', name="light1", color="1 1 1", position="0 0 800", cutoff="25", exponent="1", drawSource="false")
    rootNode.addObject('SpotLight', name="light2", color="1 1 1", position="800 0 0", direction="-1 0 0", cutoff="25", exponent="1", drawSource="false")
    rootNode.addObject('SpotLight', name="light3", color="1 1 1", position="-800 0 0", direction="1 0 0", cutoff="25", exponent="1", drawSource="false")

    Robot(rootNode)

    return rootNode