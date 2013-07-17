function pOut=orderPointCW(pIn)
% pIn can be 2*N or 3*N array
x = pIn(1,:);
y = pIn(2,:);
% Step 1: Find the centroid:

cx = mean(x);
cy = mean(y);
% Step 2: Find the angles:

a = atan2(y - cy, x - cx);
% Step 3: Find the correct sorted order:

[~, order] = sort(a);
% Step 4: Reorder the coordinates:

pOut = pIn(:,order);
