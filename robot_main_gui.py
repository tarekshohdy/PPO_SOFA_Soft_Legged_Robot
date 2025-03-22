# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key
from stlib3.physics.deformable import ElasticMaterialObject
from softrobots.actuators import PullingCable
from stlib3.physics.collision import CollisionMesh
from splib3.loaders import loadPointListFromFile
import Sofa.Gui

def main():
    import SofaRuntime
    import Sofa.Gui
    SofaRuntime.importPlugin("SofaOpenglVisual")
    SofaRuntime.importPlugin("SofaImplicitOdeSolver")

    root = Sofa.Core.Node("root")
    createScene(root)
    Sofa.Simulation.init(root)
    Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()

class TimerController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.use_sofa_profiler_timer = False

    def onAnimateBeginEvent(self, event):
        pass

    def onAnimateEndEvent(self, event):
        pass

class FingerController1_1(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.KP_1:
            displacement += 1.

        elif e["key"] == Key.Q:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]

class FingerController1_2(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.KP_2:
            displacement += 1.

        elif e["key"] == Key.W:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]

class FingerController1_3(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.KP_3:
            displacement += 1.

        elif e["key"] == Key.E:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]

class FingerController2_1(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.KP_4:
            displacement += 1.

        elif e["key"] == Key.R:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]

class FingerController2_2(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.KP_5:
            displacement += 1.

        elif e["key"] == Key.T:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]

class FingerController2_3(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.KP_6:
            displacement += 1.

        elif e["key"] == Key.Y:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]

class FingerController3_1(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.KP_7:
            displacement += 1.

        elif e["key"] == Key.U:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]

class FingerController3_2(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.KP_8:
            displacement += 1.

        elif e["key"] == Key.I:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]

class FingerController3_3(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.KP_9:
            displacement += 1.

        elif e["key"] == Key.P:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]

def Robot(parentNode=None, name="Robot",
           rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0]):

    robot = parentNode.addChild(name)
    eobject = ElasticMaterialObject(robot,
                                    volumeMeshFileName="mesh/robot_50mm.vtk",
                                    poissonRatio=0.0821706941,
                                    youngModulus=15997.6155,
                                    totalMass=2,
                                    surfaceColor=[1.0, 1.0, 1.0, 1.0],
                                    surfaceMeshFileName="mesh/robot_50mm.stl",
                                    rotation=rotation,
                                    translation=translation,
                                    name="robot")

    robot.addChild(eobject)

    CollisionMesh(eobject, name="CollisionMesh",
                  surfaceMeshFileName="mesh/robot_50mm.stl",
                  rotation=rotation, translation=translation,
                  collisionGroup=[1, 2])

    cable1_1 = PullingCable(eobject,
                         "PullingCable1.1",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, 0.0],
                         translation=[-57.74, 100.0, 0.0],
                         cableGeometry=loadPointListFromFile("mesh/string.json"));

    eobject.addObject(FingerController1_1(cable1_1))

    cable1_2 = PullingCable(eobject,
                         "PullingCable1.2",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, 120.0],
                         translation=[-57.74, 100.0, 0.0],
                         cableGeometry=loadPointListFromFile("mesh/string.json"));

    eobject.addObject(FingerController1_2(cable1_2))

    cable1_3 = PullingCable(eobject,
                         "PullingCable1.3",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, -120.0],
                         translation=[-57.74, 100.0, 0.0],
                         cableGeometry=loadPointListFromFile("mesh/string.json"));

    eobject.addObject(FingerController1_3(cable1_3))

    cable2_1 = PullingCable(eobject,
                         "PullingCable2.1",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, 0.0],
                         translation=[-57.74, -100.0, 0.0],
                         cableGeometry=loadPointListFromFile("mesh/string.json"));
    
    eobject.addObject(FingerController2_1(cable2_1))
    
    cable2_2 = PullingCable(eobject,
                         "PullingCable2.2",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, 120.0],
                         translation=[-57.74, -100.0, 0.0],
                         cableGeometry=loadPointListFromFile("mesh/string.json"));
    
    eobject.addObject(FingerController2_2(cable2_2))
    
    cable2_3 = PullingCable(eobject,
                         "PullingCable2.3",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, -120.0],
                         translation=[-57.74, -100.0, 0.0],
                         cableGeometry=loadPointListFromFile("mesh/string.json"));

    eobject.addObject(FingerController2_3(cable2_3))

    cable3_1 = PullingCable(eobject,
                         "PullingCable3.1",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, 0.0],
                         translation=[115.47, 0.0, 0.0],
                         cableGeometry=loadPointListFromFile("mesh/string.json"));

    eobject.addObject(FingerController3_1(cable3_1))

    cable3_2 = PullingCable(eobject,
                         "PullingCable3.2",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, 120.0],
                         translation=[115.47, 0.0, 0.0],
                         cableGeometry=loadPointListFromFile("mesh/string.json"));

    eobject.addObject(FingerController3_2(cable3_2))

    cable3_3 = PullingCable(eobject,
                         "PullingCable3.3",
                         pullPointLocation=None,
                         rotation=[0.0, -90.0, -120.0],
                         translation=[115.47, 0.0, 0.0],
                         cableGeometry=loadPointListFromFile("mesh/string.json"));

    eobject.addObject(FingerController3_3(cable3_3))
    
    totalMass = 1.0
    volume = 1.0
    inertiaMatrix=[1., 0., 0., 0., 1., 0., 0., 0., 1.]

    floor = robot.addChild("floor")

    floor.addObject('MechanicalObject', name="mstate", template="Rigid3", translation2=[0.0, 0.0, -200.0], rotation2=[90., 0., 0.], showObjectScale=1.0)
    floor.addObject('UniformMass', name="mass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    floorCollis = floor.addChild('collision')
    floorCollis.addObject('MeshOBJLoader', name="loader", filename="mesh/floor.obj", triangulate="true", scale3d=[5.0]*3)
    floorCollis.addObject('MeshTopology', src="@loader")
    floorCollis.addObject('MechanicalObject')
    floorCollis.addObject('TriangleCollisionModel', moving=False, simulated=False)
    floorCollis.addObject('LineCollisionModel', moving=False, simulated=False)
    floorCollis.addObject('PointCollisionModel', moving=False, simulated=False)
    floorCollis.addObject('RigidMapping')

    floorVisu = floor.addChild("VisualModel")
    floorVisu.loader = floorVisu.addObject('MeshOBJLoader', name="loader", filename="mesh/floor.obj")
    floorVisu.addObject('OglModel', name="model", src="@loader", scale3d=[5.0]*3, color=[0.6, 0.1, 0.2], updateNormals=False)
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

    MainHeader(rootNode, gravity=[0.0, 0.0, -20000.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=0.1*100, contactDistance=0.05*100, frictionCoef=20)
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"
    rootNode.addObject( TimerController() )

    rootNode.addObject('LightManager', )
    rootNode.addObject('SpotLight', name="light1", color="1 1 1", position="0 0 800", cutoff="25", exponent="1", drawSource="false")
    rootNode.addObject('SpotLight', name="light2", color="1 1 1", position="800 0 0", direction="-1 0 0", cutoff="25", exponent="1", drawSource="false")
    rootNode.addObject('SpotLight', name="light3", color="1 1 1", position="-800 0 0", direction="1 0 0", cutoff="25", exponent="1", drawSource="false")

    Robot(rootNode)

    return rootNode

if __name__ == '__main__':
    main()