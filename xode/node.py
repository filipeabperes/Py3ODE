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
XODE Parse Tree
@author: U{Timothy Stranex<mailto:timothy@stranex.com>}
"""

from . import transform

class AncestorNotFoundError(Exception):
    """
    Raised when an ancestor represeting an ODE object of some type was not
    found in the tree.

    @ivar type: The object type.
    @type type: class
    """
    
    def __init__(self, type):
        self.type = type
        
    def __str__(self):
        return "<AncestorNotFoundException: " \
               "No ancestor with type %s found.>" % repr(self.type.__name__)

class TreeNode:
    """
    A node in an XODE parse tree.
    """

    def __init__(self, name, parent):
        """
        Initialises this node. If the parent is not C{None}, parent.addChild()
        is called.

        @param name: The name of this container or C{None} if there is none.
        @type name: str

        @param parent: The parent of this node or C{None}.
        @type parent: instance or C{None}
        """

        self._name = name
        self._parent = parent
        self._obj = None
        self._transform = transform.Transform()
        
        self._childs = []
        self._namedChild = {}

        if (self._parent is not None):
            self._parent.addChild(self, name)

    def takeParser(self, parser):
        """
        Called to make this node handle further parsing. It will release the
        parser when it has finished.

        @param parser: The parser.
        @type parser: instance of L{parser.Parser}
        """

    def setODEObject(self, obj):
        """
        Sets the ODE object represented by this node.

        @param obj: The ODE object.
        @type obj: instance
        """

        self._obj = obj

    def getODEObject(self):
        """
        @return: The ODE object represented by this node. C{None} is returned
        if this node does not represent an ODE object.
        @rtype: instance
        """

        return self._obj

    def setNodeTransform(self, transform):
        """
        @param transform: This node's transform.
        @type transform: instance of L{transform.Transform}
        """

        self._transform = transform

    def getNodeTransform(self):
        """
        @return: The transform of this node.
        @rtype: instance of L{transform.Transform}
        """

        return self._transform

    def getTransform(self, untilAncestor=None):
        """
        Calculates the absolute transform at this node. It calculates the
        transforms recursively from the root node. If C{untilAncestor} is
        passed, the transform is calculated relative to it. If C{untilAncestor}
        is passed but is not an ancestor of this node, the transform is
        calculated from the root node as if C{None} was passed.

        @param untilAncestor: The ancestor to calculate the transform from.
        @type untilAncestor: instance of L{TreeNode}
        
        @return: The absolute transform at this node.
        @rtype: instance of L{transform.Transform}
        """

        p = self.getParent()
        t = self.getNodeTransform()

        if ((p is None) or (t.isAbsolute()) or (p is untilAncestor)):
            return t
        else:
            return p.getTransform(untilAncestor) * t

    def getName(self):
        """
        @return: This node's name. If it is not named, C{None} is returned.
        @rtype: str or C{None}
        """

        return self._name

    def getChildren(self):
        """
        @return: The list of child nodes.
        @rtype: list
        """

        return self._childs

    def namedChild(self, name):
        """
        Retrieves a named child node. If no child by that name is a direct
        child of this node, all the child nodes are searched recursively until
        either the named child is found or every node has been searched.

        @param name: The name of the object.
        @type name: str
        
        @return: The node.
        @rtype: instance of L{TreeNode}

        @raise KeyError: If no node with the given name was found.
        """

        if (name in self._namedChild):
            return self._namedChild[name]
        else:
            for child in self._childs:
                if (isinstance(child, TreeNode)):
                    try:
                        obj = child.namedChild(name)
                    except KeyError:
                        pass
                    else:
                        return obj

        raise KeyError("Could not find child named '%s'." % name)

    def addChild(self, child, name):
        """
        Adds a child node.

        @param child: The child node.
        @type child: instance of L{TreeNode}

        @param name: The child's name. If the child is not named, pass C{None}.
        @type name: str or C{None}
        """

        if (name is not None):
            self._namedChild[name] = child

        self._childs.append(child)

    def getParent(self):
        """
        @return: The parent of this node. C{None} is returned if there is no
        parent node.
        @rtype: instance of L{TreeNode} or C{None}
        """

        return self._parent

    def getFirstAncestor(self, type):
        """
        Find the first ancestor of this node that represents an ODE object of
        the specified type.

        @param type: The ODE type.
        @type type: class

        @return: The ancestor node.
        @rtype: instance of L{TreeNode}

        @raise AncestorNotFoundError: If no ancestor matching the criteria was
        found.
        """

        parent = self.getParent()
        if (parent is not None):
            if (isinstance(parent.getODEObject(), type)):
                return parent
            else:
                return parent.getFirstAncestor(type)
        else:
            raise AncestorNotFoundError(type)

    def getRoot(self):
        """
        Finds the root node of this parse tree.

        @return: The root node.
        @rtype: instance of L{TreeNode}
        """

        if (self.getParent() is None):
            return self
        else:
            return self.getParent().getRoot()
