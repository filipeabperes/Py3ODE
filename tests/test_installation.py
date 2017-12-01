#!/usr/bin/env python

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


import unittest
import math
import subprocess
from xode import node, transform, parser, errors


class TestOdeInstall(unittest.TestCase):
    def setUp(self):
        pass

    def testOdeInstall(self):
        subprocess.check_call('./install_ode.sh')


if (__name__ == '__main__'):
     unittest.main()
