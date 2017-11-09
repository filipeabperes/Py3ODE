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
XODE Importer for PyODE

Introduction
============

This module is part of the implementation of an
U{XODE<http://tankammo.com/xode/>} importer. It is part of the PyODE package.
The parser uses the C{xml.parsers.expat} module.

The parser creates a tree from an XODE document which the programmer can query
or transverse. Many of the tree nodes correspond to ODE objects but some are
present merely for organisation (such as the Group node).

Currently, the following features of XODE are not supported:

    - Quaternion and axis-angle rotation modes
    - Groups
    - Joints other than BallJoint
    - Extension support

Usage
=====

Here's an example showing how to parse a document and transverse the resultant
tree::

  | from xode import parser
  | import ode
  |
  | f = file('xode-document.xml')
  | p = parser.Parser(f)
  | root = p.parseFile(f)
  |
  | # Retrieve named objects
  | body1 = root.childNamed('body1').getODEObject()
  | joint1 = root.childNamed('joint1').getODEObject()
  |
  | # Transverse the tree printing object names
  | def transverse(node):
  |     print node.getName()
  |     for c in node.getChildren():
  |         transverse(c)
  | transverse(root)

@see: L{node.TreeNode}
@author: U{Timothy Stranex<mailto:timothy@stranex.com>}
"""

import ode
import xml.parsers.expat
from . import errors, transform, node, body, joint, geom

class Parser:
    """
    An XODE parser.

    Parameters
    ==========

    Certain aspects of the parsing can be controlled by setting parameters with
    L{setParams()}.
    
    C{spaceFactory}
    ---------------

    C{spaceFactory} can be set to a callable object that creates instances
    which inherit from L{ode.SpaceBase}. This factory will be used by the
    parser to create Space objects for <space> tags unless the class is
    overridden in the XODE file. The default behavior is to use the
    L{ode.SimpleSpace} class.

    Example using L{ode.HashSpace}::

        | from xode import parser
        | import ode
        |
        | p = parser.Parser()
        | p.setParams(spaceFactory=ode.HashSpace)

    Example using L{ode.QuadTreeSpace}::

        | from xode import parser
        | import ode
        |
        | def makeSpace():
        |     return ode.QuadTreeSpace((0, 0, 0), (2, 2, 2), 3)
        |
        | p = parser.Parser()
        | p.setParams(spaceFactory=makeSpace)
    """

    def __init__(self):
        """
        Initialise the parser.
        """

        self._params = {}
        self.setParams(spaceFactory=ode.SimpleSpace)

    def _nullHandler(self, *args, **kwargs):
        return

    def push(self, startElement=None, endElement=None):
        self._handlers.append({'startElement': self._expat.StartElementHandler,
                               'endElement': self._expat.EndElementHandler})
        
        self._expat.StartElementHandler = startElement or self._nullHandler
        self._expat.EndElementHandler = endElement or self._nullHandler

    def pop(self):
        top = self._handlers.pop()
        self._expat.StartElementHandler = top['startElement']
        self._expat.EndElementHandler = top['endElement']

    def _create(self):
        """
        Creates an expat parser.
        """

        self._handlers = []
        self._expat = xml.parsers.expat.ParserCreate()

        self._expat.StartElementHandler = self._nullHandler
        self._expat.EndElementHandler = self._nullHandler

        self.push(self._startElement)
        
        return self._expat

    def _startElement(self, name, attrs):
        if (name == 'xode'):
            self._root = Root(None, None)
            self._root.takeParser(self)
        else:
            raise errors.InvalidError('Root element must be <xode>.')

    def parseVector(self, attrs):
        """
        Parses an element's attributes as a vector.
        
        @return: The vector (x, y, z).
        @rtype: tuple

        @raise errors.InvalidError: If the attributes don't correspond to a
        valid vector.
        """
        
        try:
            vec = float(attrs['x']), float(attrs['y']), float(attrs['z'])
        except ValueError:
            raise errors.InvalidError('Vector attributes must be numbers.')
        except KeyError:
            raise errors.InvalidError('Vector must have x, y and z attributes.')
        else:
            return vec

    def parseString(self, xml):
        """
        Parses the given string.

        @param xml: The string to parse.
        @type xml: str

        @return: The root container.
        @rtype: instance of L{node.TreeNode}

        @raise errors.InvalidError: If document is invalid.
        """

        self._create().Parse(xml, 1)
        return self._root

    def parseFile(self, fp):
        """
        Parses the given file.

        @param fp: A file-like object.
        @type fp: file-like instance

        @return: The root container.
        @rtype: instance of L{node.TreeNode}

        @raise errors.InvalidError: If document is invalid.
        """

        self._create().ParseFile(fp)
        return self._root

    def setParams(self, **params):
        """
        Sets some parse parameters.
        """

        self._params.update(params)

    def getParam(self, name):
        """
        @param name: The parameter name.
        @type name: str

        @return: The value of the given parameter.

        @raise KeyError: If the parameter is not defined.
        """

        return self._params[name]

class Root(node.TreeNode):
    """
    The root of the object structure. It corresponds to the <xode> tag.
    """

    def takeParser(self, parser):
        """
        Handles further parsing. It should be called immediately after the
        <xode> tag is encountered.

        @param parser: The parser.
        @type parser: instance of L{Parser}
        """
        
        self._parser = parser
        self._parser.push(startElement=self._startElement,
                          endElement=self._endElement)

    def _startElement(self, name, attrs):
        nodeName = attrs.get('name', None)
        
        if (name == 'world'):
            world = World(nodeName, self)
            world.takeParser(self._parser)
        elif (name == 'ext'):
            pass
        else:
            raise errors.ChildError('xode', name)

    def _endElement(self, name):
        if (name == 'xode'):
            self._parser.pop()
        
class World(node.TreeNode):
    """
    Represents an ode.World object. It corresponds to the <world> tag.
    """

    def __init__(self, name, parent):
        node.TreeNode.__init__(self, name, parent)
        self.setODEObject(ode.World())

    def takeParser(self, parser):
        """
        Handles further parsing. It should be called immediately after the
        <world> tag is encountered.

        @param parser: The parser.
        @type parser: instance of L{Parser}
        """
        
        self._parser = parser
        self._parser.push(startElement=self._startElement,
                          endElement=self._endElement)

    def _startElement(self, name, attrs):
        nodeName = attrs.get('name', None)
        
        if (name == 'transform'):
            t = transform.Transform()
            t.takeParser(self._parser, self, attrs)
        elif (name == 'space'):
            space = Space(nodeName, self)
            space.takeParser(self._parser)
        elif (name == 'ext'):
            pass
        else:
            raise errors.ChildError('world', name)

    def _endElement(self, name):
        if (name == 'world'):
            self._parser.pop()

class Space(node.TreeNode):
    """
    Represents an ode.Space object and corresponds to the <space> tag.
    """

    def __init__(self, name, parent):
        node.TreeNode.__init__(self, name, parent)

    def takeParser(self, parser):
        """
        Handles further parsing. It should be called immediately after the
        <space> tag is encountered.

        @param parser: The parser.
        @type parser: instance of L{Parser}
        """

        self.setODEObject(parser.getParam('spaceFactory')())
        
        self._parser = parser
        self._parser.push(startElement=self._startElement,
                          endElement=self._endElement)

    def _startElement(self, name, attrs):
        nodeName = attrs.get('name', None)
        
        if (name == 'transform'):
            t = transform.Transform()
            t.takeParser(self._parser, self, attrs)
        elif (name == 'geom'):
            g = geom.Geom(nodeName, self)
            g.takeParser(self._parser)
        elif (name == 'group'):
            # parse group
            pass
        elif (name == 'body'):
            b = body.Body(nodeName, self, attrs)
            b.takeParser(self._parser)
        elif (name == 'jointgroup'):
            # parse joint group
            pass
        elif (name == 'joint'):
            j = joint.Joint(nodeName, self)
            j.takeParser(self._parser)
        elif (name == 'ext'):
            # parse ext
            pass
        else:
            raise errors.ChildError('space', name)

    def _endElement(self, name):
        if (name == 'space'):
            self._parser.pop()
