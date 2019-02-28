#!/usr/bin/env python3

"""
    Py3ODE example 1

    Create a plane and a ball somwhere higher, let the ball fall, bounce, end.
"""

import faulthandler
import ode
import sys

class Tutorial1:
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
        world.setGravity((0,0,-10)) # ODE uses Z as height by default

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
        body.setPosition((0, 0, 10))
        body.setMovedCallback(self.body_callback)
        # collision geom
        geom = ode.GeomSphere(space, 1)
        geom.setBody(body)

        # plane
        plane = ode.GeomPlane(space)

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
    #module = Tutorial1(True)
    module = Tutorial1()
    module.run_simulation()
