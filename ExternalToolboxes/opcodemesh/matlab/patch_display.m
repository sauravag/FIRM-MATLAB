function h = patch_display( fv ,showfacecolor, showtexture,varargin)
%SHOWMESH Summary of this function goes here
%   Detailed explanation goes here

V = fv.vertices;
F = fv.faces;

pmax = max(fv.vertices);
pmin = min(fv.vertices);
pabs = 1*max(abs(pmax),abs(pmin));
pmax = pmax + pabs;
pmin = pmin - pabs;

if nargin<2
    showfacecolor=0;
end
if nargin>=4 %% get color from, the user
    facecolor = [varargin{1}];
else
    facecolor = [.275 .384 .165];
end
if nargin>=5 %% do not change the figure lmits
    plimFlag = [varargin{2}];
else
    plimFlag = 'singleObject';
end
if nargin<3,
    showtexture = 0;
end
if ~showfacecolor
    h = trisurf(F,V(:,1),V(:,2),V(:,3),'facecolor',facecolor,'edgecolor','none');
else
    h = trisurf(F,V(:,1),V(:,2),V(:,3),'facecolor',facecolor,'edgecolor',[1 0 0]);
end
axis equal;
view([ 0 0 100]);
axis vis3d;
% zoom(1.5);
switch plimFlag
    case 'singleObject'
        axis([pmin(1) pmax(1) pmin(2) pmax(2) pmin(3) pmax(3)]);
        
    case 'drawInMap'
        plims = [user_data_class.par.sim.env_limits -1 3 ];
        axis(plims);
        
end
% camlight('headlight');
% camlight;
% material shiny;
material dull
if showtexture && isfield(fv,'textureimage'),
    img(:,:,1) = adapthisteq(fv.textureimage(:,:,1));
    img(:,:,2) = adapthisteq(fv.textureimage(:,:,2));
    img(:,:,3) = adapthisteq(fv.textureimage(:,:,3));
    %img = decorrstretch(fv.textureimage);
    patch_texture(h,img,fv.texturevertices);
end
rotate3d on;

