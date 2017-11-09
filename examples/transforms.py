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

# PyODE Example: Transforms
# This example demonstrates the way object transforms are calculated relative
# to the parent element's transform in XODE.

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from cgtypes import *

import pygame
import math
import ode
import xode.parser

doc = '''<?xml version="1.0" encoding="iso-8859-1"?>
<xode>
  <world name="world1">
    <space name="space1">

      <body>
        <transform>
          <position x="0" y="0" z="0"/>
          <rotation>
            <euler x="0" y="0" z="0" aformat="degrees"/>
          </rotation>
        </transform>

        <!-- Y-axis Rotations -->

        <body>
          <transform scale="1.25">
            <position x="0" y="1" z="0"/>
            <rotation>
              <euler x="0" y="30" z="0" aformat="degrees"/>
            </rotation>
          </transform>

          <body>
            <transform scale="1.25">
              <position x="0" y="1" z="0"/>
              <rotation>
                <euler x="0" y="30" z="0" aformat="degrees"/>
              </rotation>
            </transform>
          </body>
          
        </body>

        <!-- X-axis Rotations -->

        <body>
          <transform scale="1.25">
            <position x="1" y="0" z="0"/>
            <rotation>
              <euler x="30" y="0" z="0" aformat="degrees"/>
            </rotation>
          </transform>

          <body>
            <transform scale="1.25">
              <position x="1" y="0" z="0"/>
              <rotation>
                <euler x="30" y="0" z="0" aformat="degrees"/>
              </rotation>
            </transform>
          
          </body>
        </body>

        <!-- Z-axis Rotations -->

        <body>
          <transform scale="1.25">
            <position x="0" y="0" z="-1"/>
            <rotation>
              <euler x="0" y="0" z="30" aformat="degrees"/>
            </rotation>
          </transform>

          <body>
            <transform scale="1.25">
              <position x="0" y="0" z="-1"/>
              <rotation>
                <euler x="0" y="0" z="30" aformat="degrees"/>
              </rotation>
            </transform>
          
          </body>
        </body>
        
      </body>

    </space>
  </world>
</xode>
'''

def prepare_GL(c):
    """Prepare drawing.
    """
    
    # Viewport
    glViewport(0, 0, 640, 480)

    # Initialize
    glClearColor(0.8, 0.8, 0.9, 0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST)
    glDisable(GL_LIGHTING)
    glEnable(GL_LIGHTING)
    glEnable(GL_NORMALIZE)
    glShadeModel(GL_FLAT)

    # Projection
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    P = mat4(1).perspective(45,1.3333,0.2,20)
    glMultMatrixd(P.toList())

    # Initialize ModelView matrix
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    # Light source
    glLightfv(GL_LIGHT0,GL_POSITION,[0,0,1,0])
    glLightfv(GL_LIGHT0,GL_DIFFUSE,[1,1,1,1])
    glLightfv(GL_LIGHT0,GL_SPECULAR,[1,1,1,1])
    glEnable(GL_LIGHT0)

    # View transformation
    V = mat4(1).lookAt(1.2*vec3(0.5*c,0.7*c,c),(1.0,1.0,0), up=(0,1,0))
    V.rotate(math.pi,vec3(0,1,0))
    V = V.inverse()
    glMultMatrixd(V.toList())

def draw_body(body):
    """Draw an ODE body.
    """
    
    x,y,z = body.getPosition()
    R = body.getRotation()
    T = mat4()
    T[0,0] = R[0]
    T[0,1] = R[1]
    T[0,2] = R[2]
    T[1,0] = R[3]
    T[1,1] = R[4]
    T[1,2] = R[5]
    T[2,0] = R[6]
    T[2,1] = R[7]
    T[2,2] = R[8]
    T[3] = (x,y,z,1.0)

    glPushMatrix()
    glMultMatrixd(T.toList())
    if body.shape=="box":
        sx,sy,sz = body.boxsize
        glScale(sx, sy, sz)
        glutSolidCube(1)
    glPopMatrix()

######################################################################

# Initialize pygame
passed, failed = pygame.init()

# Open a window
srf = pygame.display.set_mode((640,480), pygame.OPENGL | pygame.DOUBLEBUF)

root = xode.parser.Parser().parseString(doc)

world = root.namedChild('world1').getODEObject()
world.setGravity( (0, 0, 0) )


# Add all ODE bodies from the XODE document into bodies.
def transverse(node):
    obj = node.getODEObject()
    if (isinstance(obj, ode.Body)):
        # Set attributes for draw_body()
        obj.shape = 'box'
        obj.boxsize = (0.4, 0.4, 0.4)
        bodies.append(obj)
        
    for node in node.getChildren():
        transverse(node)

bodies = []
transverse(root)

# Some variables used inside the simulation loop
fps = 50
dt = 1.0/fps
counter = 0.0
running = True
clk = pygame.time.Clock()
while running:
    events = pygame.event.get()
    for e in events:
        if e.type==pygame.QUIT:
            running=False

    if (counter < 5):
        counter = counter + 0.1

    # Draw the scene
    prepare_GL(counter)
    for b in bodies:
        draw_body(b)
    
    pygame.display.flip()

    # Simulate
    n = 2
    for i in range(n):
        world.step(dt/n)
    
    clk.tick(fps)
