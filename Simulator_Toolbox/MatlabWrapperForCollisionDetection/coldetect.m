% COLDETECT Collision detection
%   T = COLDETECT(TRI1, TRI2, TRANS1, TRANS2) returns the index of the
%   first transformation that involves a collision between two objects. The
%   two objects are defined by the triangles specified in TRI1 and TRI2.
%   The matrices TRI1 and TRI2 have nine columns, defining the vertices of
%   the triangles (x1, y1, z1, x2, y2, z2, x3, y3, z3). The matrices TRANS1
%   and TRANS2 define the transformations of objects. They both have twelve
%   columns and have the same number of rows.
%
%   This function uses the V-Collide library developed at the University of
%   North Carolina. Further information about this library may be obtained
%   from http://www.cs.unc.edu/~geom/V_COLLIDE
%
%   Mykel J. Kochenderfer
%   23 January 2008
%   MIT Lincoln Laboratory