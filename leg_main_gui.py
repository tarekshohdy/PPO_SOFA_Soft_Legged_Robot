# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
from softrobots.actuators import PullingCable
from stlib3.physics.collision import CollisionMesh
from splib3.loaders import loadPointListFromFile
import Sofa.Gui

path = __file__
path = path[:path.rfind("/")]

def main():
    import SofaRuntime
    import Sofa.Gui
    SofaRuntime.importPlugin("SofaOpenglVisual")
    SofaRuntime.importPlugin("SofaImplicitOdeSolver")
    
    createScene(root)
    Sofa.Simulation.init(root)
    Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()

class FingerController1(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.J:
            displacement += 1.

        elif e["key"] == Key.U:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]
        
class FingerController2(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.K:
            displacement += 1.

        elif e["key"] == Key.I:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]

class FingerController3(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.L:
            displacement += 1.

        elif e["key"] == Key.P:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]

def Leg(parentNode=None, name="Leg",
           rotation=[90.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[30.0, -30.0, 5.0, -30.0, 30.0, -10.0], pullPointLocation=[0.0, 0.0, 0.0]):
    leg = parentNode.addChild(name)
    eobject = ElasticMaterialObject(leg,
                                    volumeMeshFileName=path + "/mesh/leg_coarse.vtk",
                                    poissonRatio=0.0821706941,
                                    youngModulus=15997.6155,
                                    totalMass=0.3,
                                    surfaceColor=[0.0, 0.8, 0.7, 1.0],
                                    surfaceMeshFileName=path + "/mesh/leg.stl",
                                    rotation=rotation,
                                    translation=translation,
                                    name="leg")

    leg.addChild(eobject)

    CollisionMesh(eobject, name="CollisionMesh",
                  surfaceMeshFileName=path + "/mesh/leg.stl",
                  rotation=rotation, translation=translation,
                  collisionGroup=[1, 2])

    FixedBox(eobject, atPositions=fixingBox, doVisualization=True)

    cable1 = PullingCable(eobject,
                         "PullingCable1",
                         pullPointLocation=pullPointLocation,
                         rotation=[0.0, 90.0, -90.0],
                         translation=[0.0, 0.0, 0.0],
                         cableGeometry=loadPointListFromFile(path + "/mesh/string.json"));

    eobject.addObject(FingerController1(cable1))

    cable2 = PullingCable(eobject,
                         "PullingCable2",
                         pullPointLocation=pullPointLocation,
                         rotation=[120.0, 90.0, -90.0],
                         translation=[0.0, 0.0, 0.0],
                         cableGeometry=loadPointListFromFile(path + "/mesh/string.json"));

    eobject.addObject(FingerController2(cable2))

    cable3 = PullingCable(eobject,
                         "PullingCable3",
                         pullPointLocation=pullPointLocation,
                         rotation=[-120.0, 90.0, -90],
                         translation=[0.0, 0.0, 0.0],
                         cableGeometry=loadPointListFromFile(path + "/mesh/string.json"));

    eobject.addObject(FingerController3(cable3))

    return leg


def createScene(rootNode):
    from stlib3.scene import MainHeader, ContactHeader
    rootNode.dt = 0.01

    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Algorithm", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Intersection", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.AnimationLoop", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Lagrangian.Correction", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.IO.Mesh", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Direct", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.SolidMechanics.FEM.Elastic", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Engine.Select", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.GL.Component.Shader")
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Projective")
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    
    MainHeader(rootNode, gravity=[0.0, 0.0, -981.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, frictionCoef=0.08)
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"

    rootNode.addObject('LightManager', )
    rootNode.addObject('SpotLight', name="light1", color="1 1 1", position="0 0 300", cutoff="25", exponent="1", drawSource="false")
    rootNode.addObject('SpotLight', name="light2", color="1 1 1", position="300 0 90", direction="-1 0 0", cutoff="25", exponent="1", drawSource="false")

    Leg(rootNode)

    return rootNode

if __name__ == '__main__':
    root = Sofa.Core.Node("root")
    main()