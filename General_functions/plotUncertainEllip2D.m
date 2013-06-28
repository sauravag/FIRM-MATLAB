function h_ellps=plotUncertainEllip2D(C,nu,spec, line_width, magnify)
% function h_ellps=plotUncertainEllip2D(C,nu,color)

%hold on;
if nargin > 5
    error('maximum number of input arguments are 4');   % note that two of input arguments are not inside the "varargin".
end
if (nargin < 5) || isempty(magnify)
    magnify = 1;  %Ali has added and should be removed as soon as possible. Making it equal to 1 is the same as removing it.
end
if (nargin < 4) || isempty(line_width)
    line_width = 1;
end
if (nargin < 3) || isempty(spec)
    spec = 'b';
end

chi2inv_table=[0.95 2 5.99146454710798;
    0.95 3 7.81472790325116;
    0.95 6 12.59158724374398
    0.99 2 9.21034
    0.99 3 11.34487
    0.99 6 16.81189
    0.995 2 10.59663
    0.995 3 12.83816
    0.995 6 18.54758
    0.75 2 2.77259
    0.75 3 4.10834
    0.75 6 7.84080];

dim = length(nu);  % I think this is slightly wrong!! since the dimension of the whole state should matter, not the dimension that is inputted here for the plotting (which is mostly 2)
confidence_level = 0.99;
chi2 = chi2inv_table(chi2inv_table(:,1)==confidence_level & chi2inv_table(:,2)==dim , 3);

chi2=9.21034; %chi2inv_table(4,3);

th=0:2*pi/100:2*pi;

x=[cos(th') sin(th')]'*sqrt(chi2)*magnify;

if(min(eig(C))<0)
    C=eye(2);
    color=[0 0 0];
    fprintf('NPSD matrix, a black false ellipse has been plot\n');
    error('NPSD matrix') % This line was not here. But since we eliminated the "color" line above, I added this to make sure we catch the wrong covariance matrices.
end
K=chol(C)';

nPoints=size(th,2);
y=K*x+[ones(1,nPoints)*nu(1);ones(1,nPoints)*nu(2)];

% h_ellps=plot(y(1,:),y(2,:),'Color',color, 'LineWidth', 1.5 );


% if strcmp(spec,'g')
%     h_ellps=plot(y(1,:),y(2,:),'color',[0 0.83 0]);disp('Dont forget you changed the color in Plot_ellipse')
% elseif strcmp(spec,'b--')
%     h_ellps=plot(y(1,:),y(2,:),'--','color',[0 0 0.7]);disp('Dont forget you changed the color in Plot_ellipse')
% else
    h_ellps=plot(y(1,:),y(2,:),spec,'LineWidth',line_width);
% end
