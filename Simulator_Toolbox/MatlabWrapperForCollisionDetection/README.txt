COLDETECT version 1.0

A simple Matlab wrapper for the V-Collide collision detection library.

Author: Mykel J. Kochenderfer
Email:  m.kochenderfer@gmail.com
Date:   25 January 2008

========
CONTENTS
========
README.txt - This file
coldetect.m - Brief documentation of coldetect function usage
coldetect.mexw32 - MEX file compiled for Win32 using Visual Studio 2005
coldetect.cpp - C++ source file 

========
BUILDING
========
A Win32 binary is provided with the V-Collide library (version 2.01) statically linked, so there is no need for 32-bit Windows users to compile from source. For those interested in building from source, follow these steps:

1. Obtain V-Collide library source from the University of North Carolina (http://www.cs.unc.edu/~geom/V_COLLIDE/).
2. Follow their instructions for building V-Collide to get VCollide.lib and RAPID.lib.
3. Copy VCollide.lib, RAPID.lib, and VCollide.H (from the V-Collide source) into a directory with coldetect.cpp.
4. Inside Matlab, run "mex coldetect.cpp VCollide.lib RAPID.lib".
5. After Matlab compiles coldetect, you should have a binary called coldetect.mex* (the rest of the file extension  depends upon the target system; for Win32 it will be coldetect.mexw32).

See the Matlab documentation on MEX files for further information.

=======
RUNNING
=======

T = COLDETECT(TRI1, TRI2, TRANS1, TRANS2) returns the index of the first transformation that involves a collision between two objects. The two objects are defined by the triangles specified in TRI1 and TRI2. The matrices TRI1 and TRI2 have nine columns, defining the vertices of the triangles (x1, y1, z1, x2, y2, z2, x3, y3, z3). The matrices TRANS1 and TRANS2 define the transformations of objects. They both have twelve columns and have the same number of rows. Each row (e1, ..., e12) defines a single transformation matrix:

[ e4  e5  e6 e1]
[ e7  e8  e9 e2]
[e10 e11 e12 e3]
[  0   0   0  1]

=========
REFERENCE
=========

T.C. Hudson, M.C. Lin, J. Cohen, S. Gottschalk, and D. Manocha. V-COLLIDE: Accelerated Collision Detection for VRML. In Proc. 2nd Annu. Sympos. on the Virtual Reality Modeling Language, Monterey, CA, USA, 1997.

=======
LICENSE
=======

The coldetect wrapper function for the V-Collide collision detection library is distributed with the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

The V-Collide library is copyright (c) University of North Carolina. This library is only freely available for NON-commercial applications. For any possible commercial use, one should contact geom@cs.unc.edu. There is a formal procedure to license the code at the nominal fees for commercial use.

