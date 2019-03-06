#!/usr/bin/env python3

"""
    Py3ODE heightmap example

    Load an image, extract its red channel, construct an ODE heightmap from the
    extracted data, add a ball somewhere higher and let it bounce on the heightmap.
"""

import faulthandler
import numpy as np # pip install numpy
import ode
import sys

from PIL import Image # pip install pillow

class Tutorial_Heightmap:
    def __init__(self, trace = False):
        self.trace = trace

    def trace_func(self, frame, event, arg):
        print("{0}, {1}, {2}".format(frame, event, arg))

    def body_callback(self, body):
        print("{0} - position: {1}, velocity: {2}, joints: {3}".format(body, body.getPosition(), body.getLinearVel(), body.getNumJoints()))

    def collision_callback(self, args, geom1, geom2):
        world, contactgroup = args
        body1, body2 = geom1.getBody(), geom2.getBody()

        if (body1 is None):
            body1 = ode.environment

        if (body2 is None):
            body2 = ode.environment

        contacts = ode.collide(geom1, geom2)
        for c in contacts:
            c.setBounce(0.5)
            c.setMu(10000)
            j = ode.ContactJoint(world, contactgroup, c)
            j.attach(body1, body2)

    def run_simulation(self):
        if self.trace:
            faulthandler.enable()
            sys.settrace(module.trace_func)

        # dynamics world
        world = ode.World()
        world.setGravity((0,-10, 0)) # heightmaps consider Y as up

        # collision space
        space = ode.SimpleSpace() # SimpleSpace is slow-ish
        # contact group
        contactgroup = ode.JointGroup()

        # mass
        M = ode.Mass()
        M.setSphere(2500, 1)
        M.mass = 1.0
        # dynamic body
        body = ode.Body(world)
        body.setMass(M)
        body.setPosition((0, 260, 0))
        body.setMovedCallback(self.body_callback)
        # collision geom
        geom = ode.GeomSphere(space, 1)
        geom.setBody(body)

        # heightmap
        h_data = ode.HeightfieldData()
        im = Image.open(sys.path[0] + "/tutorial_heightmap.png", "r")
        width, height = im.size
        pixel_values = list(im.getdata(0)) # 0 is the index of the Red channel in RGB / RGBA images - returns a list of longs for some reason tho
        height_data = np.array(pixel_values)
        height_data = height_data.astype(np.ubyte) # convert array to unsigned bytes as ODE expects so
        height_data = np.ascontiguousarray(height_data) # convert to a contiguous array so C code can read it and store data in variable to prevent garbage collection
        h_data.buildByte(height_data, False, width, height, width, height, 1, 0, 1, True)
        h_geom = ode.GeomHeightfield(data = h_data, space = space)

        # run simulation
        total_time = 0.0
        dt = 0.04
        while total_time < 4.5:
            print(total_time)
            space.collide((world, contactgroup), self.collision_callback) # collision detection
            world.quickStep(dt) # dynamics step
            contactgroup.empty()
            total_time += dt

if __name__ == '__main__':
    #module = Tutorial_Heightmap(True)
    module = Tutorial_Heightmap()
    module.run_simulation()
