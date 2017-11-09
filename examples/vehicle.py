######################################################################
# Python Open Dynamics Engine Wrapper
# Copyright (C) 2004 PyODE developers (see file AUTHORS)
# All rights reserved.
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of EITHER:
#   (1) The GNU Lesser General Public License as published by the Free
#       Software Foundation; either version 2.1 of the License, or (at
#       your option) any later version. The text of the GNU Lesser
#       General Public License is included with this library in the
#       file LICENSE.
#   (2) The BSD-style license that is included with this library in
#       the file LICENSE-BSD.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
# LICENSE and LICENSE-BSD for more details. 
######################################################################

# PyODE Example: VehicleDemo

import ode, xode.parser
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

info = """VehicleDemo

Controls:
  Left mouse button click: Apply upward force to the vehicle's chassis.
  W Key: Move forward.
  S Key: Move backward.
  A Key: Turn left.
  D Key: Turn right.
  Escape Key: Exit.
"""

# This XODE document contains the description of the vehicle, the ground plane
# and the ramp. The stack of boxes are created at runtime.

doc = '''<?xml version="1.0" encoding="iso-8859-1"?>
<xode>
  <world name="world">
    <space name="space">

      <geom name="ground">
        <plane a="0" b="1" c="0" d="-4" />
      </geom>

      <geom name="ramp">
        <transform>
          <position x="0" y="-2" z="-20" />
          <rotation>
            <euler x="-30" y="0" z="0" aformat="degrees" />
          </rotation>
        </transform>

        <box sizex="10" sizey="1" sizez="10" />
      </geom>

      <jointgroup name="vehicle_joints">

        <body name="chassis">
          <transform>
            <position x="0" y="0" z="0" />
          </transform>
          
          <geom name="chassis_geom">
            <box sizex="4" sizey="0.5" sizez="5" />
          </geom>
          
          <body name="wheel1_body">
            <transform>
              <position x="0" y="0" z="-2" />
            </transform>
            
            <geom name="wheel1_geom">
              <sphere radius="1" />
            </geom>

            <joint name="wheel1">
              <link1 body="chassis" />
              <hinge2>
                <anchor x="0" y="0" z="-2" />
                <axis x="0" y="1" z="0" FMax="50" LoStop="-0.78" HiStop="0.78"/>
                <axis x="1" y="0" z="0" FMax="5000"
                                        SuspensionERP="0.8"
                                        SuspensionCFM="0.5" />
              </hinge2>
            </joint>

            <mass>
              <mass_shape density="1">
                <sphere radius="1" />
              </mass_shape>
            </mass>
          </body>

          <body name="wheel2_body">
            <transform>
              <position x="2" y="0" z="2" />
            </transform>
            
            <geom name="wheel2_geom">
               <sphere radius="1" />
            </geom>

            <joint name="wheel2">
              <link1 body="chassis" />
              <hinge2>
                <anchor x="2" y="0" z="2" />
                <axis x="0" y="1" z="0" FMax="0" LoStop="0" HiStop="0" />
                <axis x="1" y="0" z="0" FMax="5000"
                                        SuspensionERP="0.8"
                                        SuspensionCFM="0.5" />
              </hinge2>
            </joint>

            <mass>
              <mass_shape density="1">
                <sphere radius="1" />
              </mass_shape>
            </mass>
          </body>

          <body name="wheel3_body">
            <transform>
              <position x="-2" y="0" z="2" />
            </transform>
            
            <geom name="wheel3_geom">
              <sphere radius="1" />
            </geom>

            <joint name="wheel3">
              <link1 body="chassis" />
              <hinge2>
                <anchor x="-2" y="0" z="2" />
                <axis x="0" y="1" z="0" FMax="0" LoStop="0" HiStop="0" />
                <axis x="1" y="0" z="0" FMax="5000"
                                        SuspensionERP="0.8"
                                        SuspensionCFM="0.5" />
              </hinge2>
            </joint>

            <mass>
              <mass_shape density="1">
                <sphere radius="1" />
              </mass_shape>
            </mass>
          </body>

          <mass>
            <mass_shape density="30">
              <box sizex="6" sizey="5" sizez="3" />
            </mass_shape>
          </mass>
          
        </body>
      </jointgroup>

    </space>
  </world>
</xode>
'''

class VehicleDemo:
    """
    Vehicle Demo
    """

    fps = 50.0
    cameraDistance = 10.0
    vel = 50.0
    turn = 1.0
    clip = 100.0
    res = (640, 480)

    def __init__(self):
        """
        Initialises this.
        """

        self._initOpenGL()
        self._loadObjects()
        self._buildWall()

        self._cjoints = ode.JointGroup()

        self._xRot = 0.0
        self._yRot = 0.0
        
        self._xCoeff = 360.0 / 480.0
        self._yCoeff = 360.0 / 640.0

        self._vel = 0.0
        self._turn = 0.0

    def _loadObjects(self):
        p = xode.parser.Parser()
        root = p.parseString(doc)

        self.world = root.namedChild('world').getODEObject()
        self.space = root.namedChild('space').getODEObject()
        self.ground = root.namedChild('ground').getODEObject()
        self.chassis = root.namedChild('chassis').getODEObject()
        self.wheel1 = root.namedChild('wheel1').getODEObject()
        self.wheel2 = root.namedChild('wheel2').getODEObject()
        self.wheel3 = root.namedChild('wheel3').getODEObject()

        self.root = root
        self.world.setGravity((0, -9.81, 0))

        # transverse the xode parse tree to make a list of all geoms and joints
        self._geoms = []
        self._joints = []

        def transverse(node):
            obj = node.getODEObject()
            if (isinstance(obj, ode.GeomObject)):
                self._geoms.append(obj)
            if (isinstance(obj, ode.Joint)):
                self._joints.append(obj)

            for child in node.getChildren():
                transverse(child)

        transverse(root)

    def _buildWall(self):
        """
        Create the wall of stacked boxes.
        """
        
        def makeBox(x, y):
            body = ode.Body(self.world)
            body.setPosition((x, y, -40))

            geom = ode.GeomBox(self.space, lengths=(0.5, 0.5, 0.5))
            geom.setBody(body)
            
            self._geoms.append(geom)

        for x in range(-4, 5):
            for y in range(7):
                makeBox(x, y)

    def _initOpenGL(self):
        """
        Initialise the scene.
        """
        
        # Create a window
        pygame.init()
        screen = pygame.display.set_mode(self.res,
                                         pygame.OPENGL | pygame.DOUBLEBUF)
        pygame.display.set_caption('PyODE Vehicle Demo')
        pygame.mouse.set_visible(False)

        glViewport(0, 0, self.res[0], self.res[1])
        glClearColor(0.8, 0.8, 0.9, 0)
        
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_NORMALIZE)
        glShadeModel(GL_FLAT)

    def _extractMatrix(self, geom):
        """
        Return a 4x4 matrix (represented by a 16-element tuple) created by
        combining the geom's rotation matrix and position.
        """
        
        x, y, z = geom.getPosition()
        rot = geom.getRotation()
        return (rot[0], rot[3], rot[6], 0.0,
                rot[1], rot[4], rot[7], 0.0,
                rot[2], rot[5], rot[8], 0.0,
                x, y, z, 1.0)

    def _renderGeom(self, geom):
        """
        Render either a ode.GeomBox or ode.GeomSphere object.
        """

        allowed = [ode.GeomBox, ode.GeomSphere]
        ok = False
        for klass in allowed:
            ok = ok or isinstance(geom, klass)
        if (not ok):
            return

        glPushMatrix()
        glMultMatrixd(self._extractMatrix(geom))

        glMaterialfv(GL_FRONT, GL_SPECULAR, (0.0, 0.0, 0.0))

        if (isinstance(geom, ode.GeomBox)):
            sx, sy, sz = geom.getLengths()
            glScale(sx, sy, sz)
            glutSolidCube(1)
        elif (isinstance(geom, ode.GeomSphere)):
            r = geom.getRadius()
            glutSolidSphere(r, 20, 20)

        glPopMatrix()

    def _renderGround(self):
        """
        Renders the ground plane.
        """

        # Draw a quad at the position of the vehicle that extends to the
        # clipping planes.

        normal, d = self.ground.getParams()
        x, y, z = self.chassis.getPosition()

        glPushMatrix()
        glTranslate(x, 0.0, z)

        glMaterialfv(GL_FRONT, GL_SPECULAR, (0.0, 1.0, 0.0))

        glBegin(GL_QUADS)
        glColor3(0.0, 1.0, 0.0)
        glNormal3f(*normal)
        glVertex3f(-self.clip, d, -self.clip)
        glNormal3f(*normal)
        glVertex3f(self.clip, d, -self.clip)
        glNormal3f(*normal)
        glVertex3f(self.clip, d, self.clip)
        glNormal3f(*normal)
        glVertex3f(-self.clip, d, self.clip)
        glEnd()

        glPopMatrix()

    def _setCamera(self):
        """
        Position the camera to C{self.cameraDistance} units behind the
        vehicle's current position and rotated depending on the mouse position.
        """

        aspect = float(self.res[0]) / float(self.res[1])
        
        x, y = pygame.mouse.get_pos()
        self._xRot = (y - self.res[1]/2) * self._xCoeff
        self._yRot = (x - self.res[0]/2) * self._yCoeff
        if (self._xRot < 0):
            self._xRot = 0
                
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glFrustum(-aspect, aspect, -1.0, 1.0, 1.5, self.clip)

        glLightfv(GL_LIGHT0, GL_POSITION, (-5.0, 10.0, 0, 0))
        glLightfv(GL_LIGHT0,GL_DIFFUSE, (1.0, 1.0, 1.0, 1.0))
        glLightfv(GL_LIGHT0,GL_SPECULAR, (1.0, 1.0, 1.0, 1.0))
        glEnable(GL_LIGHT0)
        
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # Set the camera angle to view the vehicle
        glTranslate(0.0, 0.0, -self.cameraDistance)
        glRotate(self._xRot, 1, 0, 0)
        glRotate(self._yRot, 0, 1, 0)

        # Set the camera so that the vehicle is drawn in the correct place.
        x, y, z = self.chassis.getPosition()
        glTranslate(-x, -y, -z)

    def render(self):
        """
        Render the current simulation state.
        """
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self._renderGround()

        self._setCamera()
        for geom in self._geoms:
            self._renderGeom(geom)

        glFlush()
        pygame.display.flip()

    def _keyDown(self, key):
        if (key == pygame.K_w):
            self._vel = self.vel
        elif (key == pygame.K_a):
            self._turn = -self.turn
        elif (key == pygame.K_d):
            self._turn = self.turn
        elif (key == pygame.K_s):
            self._vel = -self.vel
        elif (key == pygame.K_ESCAPE):
            self._running = False

    def _keyUp(self, key):
        if (key == pygame.K_w):
            self._vel = 0.0
        elif (key == pygame.K_a):
            self._turn = 0.0
        elif (key == pygame.K_d):
            self._turn = 0.0
        elif (key == pygame.K_s):
            self._vel = 0.0

    def doEvents(self):
        """
        Process any input events.
        """
        
        events = pygame.event.get()
        
        for e in events:
            if (e.type == pygame.QUIT):
                self._running = False
            elif (e.type == pygame.KEYDOWN):
                self._keyDown(e.key)
            elif (e.type == pygame.KEYUP):
                self._keyUp(e.key)
            elif (e.type == pygame.MOUSEBUTTONDOWN):
                if (e.button == 1):
                    self.chassis.addForce((0.0, 500000, 0.0))

    def _nearcb(self, args, geom1, geom2):
        """
        Create contact joints between colliding geoms.
        """

        body1, body2 = geom1.getBody(), geom2.getBody()
        if (body1 is None):
            body1 = ode.environment
        if (body2 is None):
            body2 = ode.environment

        if (ode.areConnected(body1, body2)):
            return

        contacts = ode.collide(geom1, geom2)

        for c in contacts:
            c.setBounce(0.2)
            c.setMu(10000)
            j = ode.ContactJoint(self.world, self._cjoints, c)
            j.attach(body1, body2)

    def run(self):
        """
        Start the demo. This method will block until the demo exits.
        """
        
        clock = pygame.time.Clock()
        self._running = True
        
        while self._running:
            self.doEvents()

            # Steering
            self.wheel1.setParam(ode.ParamVel, self._turn)
            
            # Engine
            self.wheel1.setParam(ode.ParamVel2, self._vel)
            self.wheel2.setParam(ode.ParamVel2, self._vel)
            self.wheel3.setParam(ode.ParamVel2, self._vel)

            self.space.collide((), self._nearcb)
            self.world.step(1/self.fps)
            self._cjoints.empty()
            self.render()

            # Limit the FPS.
            clock.tick(self.fps)

if (__name__ == '__main__'):
    print info
    demo = VehicleDemo()
    demo.run()
