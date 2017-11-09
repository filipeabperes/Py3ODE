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

# XODE Importer for PyODE

"""
XODE Joint Parser
@author: U{Timothy Stranex<mailto:timothy@stranex.com>}
"""

import ode
from . import node, errors

class Joint(node.TreeNode):
    """
    Represents an ode.Joint-based object and corresponds to the <joint> tag.
    """

    def __init__(self, name, parent):
        node.TreeNode.__init__(self, name, parent)

        self._world = self.getFirstAncestor(ode.World).getODEObject()

        try:
            self._jg = self.getFirstAncestor(ode.JointGroup).getODEObject()
        except node.AncestorNotFoundError:
            self._jg = None

        try:
            self._body = self.getFirstAncestor(ode.Body).getODEObject()
        except node.AncestorNotFoundError:
            self._body = None

        self._link1 = None
        self._link2 = None

        self.setODEObject(None)

    def _getName(self, name):
        root = self.getRoot()
        
        try:
            link = root.namedChild(name).getODEObject()
        except KeyError:
            raise errors.InvalidError('Joint link must reference an already '\
                                      'parsed body.')

        if (not isinstance(link, ode.Body)):
            raise errors.InvalidError('Joint link must reference a body.')

        return link

    def _getLinks(self):
        body = self._body or ode.environment

        if (self._link1 is not None):
            link1 = self._getName(self._link1)
        else:
            link1 = body
            body = ode.environment

        if (self._link2 is not None):
            link2 = self._getName(self._link2)
        else:
            link2 = body

        if (link1 is link2):
            raise errors.InvalidError('Joint requires two objects.')

        return link1, link2

    def takeParser(self, parser):
        """
        Handles further parsing. It should be called immediately after the
        <joint> tag is encountered.
        """
        
        self._parser = parser
        self._parser.push(startElement=self._startElement,
                          endElement=self._endElement)

    def _startElement(self, name, attrs):
        if (name == 'link1'):
            self._link1 = attrs['body']
        elif (name == 'link2'):
            self._link2 = attrs['body']
        elif (name == 'ext'):
            pass
        elif (name == 'amotor'):
            l1, l2 = self._getLinks()
            self._parseAMotor(self._world, l1, l2)
        elif (name == 'ball'):
            l1, l2 = self._getLinks()
            self._parseBallJoint(self._world, l1, l2)
        elif (name == 'fixed'):
            l1, l2 = self._getLinks()
            self._parseFixedJoint(self._world, l1, l2)
        elif (name == 'hinge'):
            l1, l2 = self._getLinks()
            self._parseHingeJoint(self._world, l1, l2)
        elif (name == 'hinge2'):
            l1, l2 = self._getLinks()
            self._parseHinge2Joint(self._world, l1, l2)
        elif (name == 'slider'):
            l1, l2 = self._getLinks()
            self._parseSliderJoint(self._world, l1, l2)
        elif (name == 'universal'):
            l1, l2 = self._getLinks()
            self._parseUniversalJoint(self._world, l1, l2)
        else:
            raise errors.ChildError('joint', name)

    def _endElement(self, name):
        if (name == 'joint'):
            if (self.getODEObject() is None):
                raise errors.InvalidError('No joint type element found.')
            self._parser.pop()

    def _applyAxisParams(self, joint, anum, axis):
        def setParam(name):
            attr = 'Param%s' % name
            if (anum != 0):
                attr = '%s%i' % (attr, anum+1)
            
            joint.setParam(getattr(ode, attr), float(axis[name]))

        if ('LowStop' in axis):
            axis['LoStop'] = axis['LowStop']
            del axis['LowStop']
            
        for name in list(axis.keys()):
            if (name not in ['x', 'y', 'z']):
                if (name in ['LoStop', 'HiStop', 'Vel', 'FMax', 'FudgeFactor',
                             'Bounce', 'CFM', 'StopERP', 'StopCFM',
                             'SuspensionERP', 'SuspensionCFM']):
                    setParam(name)
                else:
                    raise errors.InvalidError('Invalid attribute %s' % repr(name) +
                                              ' of <axis> element.')
        
    def _parseBallJoint(self, world, link1, link2):
        anchor = [None]
    
        def start(name, attrs):
            if (name == 'anchor'):
                anchor[0] = self._parser.parseVector(attrs)
            else:
                raise errors.ChildError('ball', name)
    
        def end(name):
            if (name == 'ball'):
                joint = ode.BallJoint(world, self._jg)
                joint.attach(link1, link2)
                if (anchor[0] is not None):
                    joint.setAnchor(anchor[0])
                
                self.setODEObject(joint)
                self._parser.pop()
    
        self._parser.push(startElement=start, endElement=end)

    def _parseFixedJoint(self, world, link1, link2):
        
        def start(name, attrs):
            raise errors.ChildError('fixed', name)
    
        def end(name):
            if (name == 'fixed'):
                self._parser.pop()
        
        joint = ode.FixedJoint(world, self._jg)
        joint.attach(link1, link2)
        self.setODEObject(joint)

        self._parser.push(startElement=start, endElement=end)

    def _parseHingeJoint(self, world, link1, link2):
        anchor = [None]
        axes = []
    
        def start(name, attrs):
            if (name == 'anchor'):
                anchor[0] = self._parser.parseVector(attrs)
            elif (name == 'axis'):
                axes.append(attrs)
            else:
                raise errors.ChildError('hinge', name)
    
        def end(name):
            if (name == 'hinge'):
                joint = ode.HingeJoint(world, self._jg)
                joint.attach(link1, link2)
                
                if (anchor[0] is not None):
                    joint.setAnchor(anchor[0])

                if (len(axes) != 1):
                    raise errors.InvalidError('Wrong number of axes for hinge'
                                              ' joint.')
                
                joint.setAxis(self._parser.parseVector(axes[0]))
                self._applyAxisParams(joint, 0, axes[0])
                
                self.setODEObject(joint)
                self._parser.pop()
    
        self._parser.push(startElement=start, endElement=end)

    def _parseSliderJoint(self, world, link1, link2):
        axes = []
    
        def start(name, attrs):
            if (name == 'axis'):
                axes.append(attrs)
            else:
                raise errors.ChildError('slider', name)
    
        def end(name):
            if (name == 'slider'):
                joint = ode.SliderJoint(world, self._jg)
                joint.attach(link1, link2)
                
                if (len(axes) != 1):
                    raise errors.InvalidError('Wrong number of axes for slider'
                                              ' joint.')
                
                joint.setAxis(self._parser.parseVector(axes[0]))
                self._applyAxisParams(joint, 0, axes[0])
                
                self.setODEObject(joint)
                self._parser.pop()
    
        self._parser.push(startElement=start, endElement=end)

    def _parseUniversalJoint(self, world, link1, link2):
        anchor = [None]
        axes = []
    
        def start(name, attrs):
            if (name == 'anchor'):
                anchor[0] = self._parser.parseVector(attrs)
            elif (name == 'axis'):
                axes.append(attrs)
            else:
                raise errors.ChildError('universal', name)
    
        def end(name):
            if (name == 'universal'):
                joint = ode.UniversalJoint(world, self._jg)
                joint.attach(link1, link2)

                if (anchor[0] is not None):
                    joint.setAnchor(anchor[0])
                
                if (len(axes) != 2):
                    raise errors.InvalidError('Wrong number of axes for '
                                              ' universal joint.')
                
                joint.setAxis1(self._parser.parseVector(axes[0]))
                self._applyAxisParams(joint, 0, axes[0])
                
                joint.setAxis2(self._parser.parseVector(axes[1]))
                self._applyAxisParams(joint, 1, axes[1])
                
                self.setODEObject(joint)
                self._parser.pop()
    
        self._parser.push(startElement=start, endElement=end)

    def _parseHinge2Joint(self, world, link1, link2):
        anchor = [None]
        axes = []
    
        def start(name, attrs):
            if (name == 'anchor'):
                anchor[0] = self._parser.parseVector(attrs)
            elif (name == 'axis'):
                axes.append(attrs)
            else:
                raise errors.ChildError('hinge2', name)
    
        def end(name):
            if (name == 'hinge2'):
                joint = ode.Hinge2Joint(world, self._jg)
                joint.attach(link1, link2)

                if (anchor[0] is not None):
                    joint.setAnchor(anchor[0])
                
                if (len(axes) != 2):
                    raise errors.InvalidError('Wrong number of axes for '
                                              ' hinge2 joint.')
                
                joint.setAxis1(self._parser.parseVector(axes[0]))
                self._applyAxisParams(joint, 0, axes[0])
                
                joint.setAxis2(self._parser.parseVector(axes[1]))
                self._applyAxisParams(joint, 1, axes[1])
                
                self.setODEObject(joint)
                self._parser.pop()
    
        self._parser.push(startElement=start, endElement=end)

    def _parseAMotor(self, world, link1, link2):
        anchor = [None]
        axes = []
    
        def start(name, attrs):
            # The XODE specification allows anchor elements for AMotor but
            # there is no way to set the anchor of an AMotor.
            
            #if (name == 'anchor'):
            #    anchor[0] = self._parser.parseVector(attrs)
            
            if (name == 'axis'):
                axes.append(attrs)
            else:
                raise errors.ChildError('amotor', name)
    
        def end(name):
            if (name == 'amotor'):
                joint = ode.AMotor(world, self._jg)
                joint.attach(link1, link2)

                if (anchor[0] is not None):
                    joint.setAnchor(anchor[0])
                
                if (len(axes) > 3):
                    raise errors.InvalidError('Wrong number of axes for '
                                              ' amotor joint.')

                joint.setNumAxes(len(axes))
                
                for i in range(len(axes)):
                    joint.setAxis(i, 0, self._parser.parseVector(axes[i]))
                    self._applyAxisParams(joint, i, axes[i])
                
                self.setODEObject(joint)
                self._parser.pop()
    
        self._parser.push(startElement=start, endElement=end)
