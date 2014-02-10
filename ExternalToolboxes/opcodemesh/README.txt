This is a Matlab wrapper for OPCODE, which is a collision detection
or ray casting library for triangular 3D meshes.

OPCODE uses a couple of different aabb trees to store the mesh and this is
a pretty simple wrapper for one of the trees.

Nice thing about opcode is that it allows for deformable meshes,
meaning that you can update the mesh while it is stored in the tree,
which is much faster than what it takes to rebuild the aabb tree.

Input and output:

To make the tree:
tree = opcodemesh(v,f);
where
vertices v : 3 x nv
faces f : 3 x nf

To intersect:
[hit,d,trix,bary] = tree.intersect(orig,dir);
where
starting points orig : 3 x nc
direction dir : 3 x nc
whether hit or not hit : nc x 1 logical
distance from orig to intersection point d : nc x 1
index into f of the intersection triangle trix : 3 x nc
barycentric coordinates of the triangle that the rays intersected bary : 2 x nc
If a ray misses, then you have 0's for trix and nan's for the rest at that index.

To get the actual points, just
Q = repmat(B(1,:),3,1).*verts(faces(trix,1),:)' + ...
    repmat(B(2,:),3,1).*verts(faces(trix,2),:)' + ...
    repmat(1-B(1,:)-B(2,:),3,1).*verts(faces(trix,3),:)';
where verts = v, faces = f, B = bary.

To update the mesh with a new set of vertices (as in deform the mesh):
tree.update(vnew);
vnew : 3 x nv
vnew must be the same size and the original set of vertices.
The topology of the mesh cannot change.

To delete the tree (which is actually done automatically by Matlab):
tree.delete();

To compile, go to ./src_matlab
and run mexall.m
which compiles the mex code and copies it to ./matlab
(I compile it against everything in opcode, so it's a bit slow.)

To run the demo, go to ./matlab
and run opcodemeshdemo.m

OPCODE is in
http://www.lia.ufc.br/~gilvan/cd/
(which is a more portable version)
and
http://www.codercorner.com/Opcode.htm
(the original site)

Also has code from
http://www.mathworks.com/matlabcentral/fileexchange/27982
for reading wavefront files for the demo
and much appreciated code from
http://www.mathworks.com/matlabcentral/fileexchange/38964
for nicely wrapping C++ functions in a Matlab class.
