function opcodemeshdemo()

% Very simple shape
% f = [1 2 3]'; v = [-1 0 0 ; 1 1 0 ; 1 -1 0]';
% [hit,d,trix,bary] = opcodemeshmex('intersect',t,1,1,[0 0 0.5]',[0 0 -1]');
 
% More complex shape
obj = read_obj('deer_bound.obj');
v = obj.vertices;
f = obj.objects(3).data.vertices;

patch_display(struct('vertices',v,'faces',f))

minb = min(v(:,[3 2]));
maxb = max(v(:,[3 2]));
%t = opcodemeshmex('create',v',f');
t = opcodemesh(v',f');


imgsize = 300;

x = (1:imgsize) ./ imgsize .* (maxb(1) - minb(1)) + minb(1);
y = (1:imgsize) ./ imgsize .* (maxb(2) - minb(2)) + minb(2);
[X,Y] = meshgrid(x,y);
Y = flipud(Y);
Z = 1000*ones(size(X));

from = [-Z(:) Y(:) X(:)]';
to = [Z(:) Y(:) X(:)]';

%[hit,d,trix,bary] = opcodemeshmex('intersect',t,from,to-from);
[hit,d,trix,bary] = t.intersect(from,to-from);
img_d = reshape(d,size(Z));
figure,imagesc(img_d);

% You can update the vertices on the fly
v=v+0.2;
%opcodemeshmex('update',t,v');
t.update(v');

%[hit,d,trix,bary] = opcodemeshmex('intersect',t,from,to-from);
[hit,d,trix,bary] = t.intersect(from,to-from);
img_d = reshape(d,size(Z));
figure,imagesc(img_d);

% opcodemeshmex('delete',t);

