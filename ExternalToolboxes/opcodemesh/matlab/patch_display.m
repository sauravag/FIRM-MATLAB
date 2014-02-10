function h = patch_display( fv ,showfacecolor, showtexture)
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
if nargin<3,
showtexture = 0;
end
    if ~showfacecolor
        h = trisurf(F,V(:,1),V(:,2),V(:,3),'facecolor',[.9 .8 .6],'edgecolor','none');
    else
        h = trisurf(F,V(:,1),V(:,2),V(:,3),'facecolor',[.9 .8 .6],'edgecolor',[1 0 0]);
    end
    axis equal, view([ 0 0 100]), axis vis3d, zoom(1.5);
    axis([pmin(1) pmax(1) pmin(2) pmax(2) pmin(3) pmax(3)]);
    camlight('headlight'); camlight right;
    material shiny;

if showtexture && isfield(fv,'textureimage'),
    img(:,:,1) = adapthisteq(fv.textureimage(:,:,1));
    img(:,:,2) = adapthisteq(fv.textureimage(:,:,2));
    img(:,:,3) = adapthisteq(fv.textureimage(:,:,3));
    %img = decorrstretch(fv.textureimage);
    patch_texture(h,img,fv.texturevertices);
end
rotate3d on;

