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
XODE Exceptions
@author: U{Timothy Stranex<mailto:timothy@stranex.com>}
"""

class InvalidError(Exception):
    """
    Raised when an XODE document is invalid.
    """

class ChildError(InvalidError):
    """
    Raised when an invalid child element is found.

    @ivar parent: The parent element.
    @type parent: str

    @ivar child: The invalid child element.
    @type child: str
    """

    def __init__(self, parent, child):
        self.parent = parent
        self.child = child

    def __str__(self):
        return '<%s> is not a valid child of <%s>.' % (self.child, self.parent)

class MissingElementError(InvalidError):
    """
    Raised when a child element is missing.

    @ivar parent: The parent element.
    @type parent: str

    @ivar child: The missing child element.
    @type child: str
    """

    def __init__(self, parent, child):
        self.parent = parent
        self.child = child

    def __str__(self):
        return 'Missing child <%s> of <%s>.' % (self.child, self.parent)
