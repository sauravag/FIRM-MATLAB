close all
clc
vRepDistnace = V_rep(2:end,:) - V_rep(1:end-1,:);
FirmDistnace = Firm(2:end,:) - Firm(1:end-1,:);
normVRep = sqrt(vRepDistnace(:,1).^2+vRepDistnace(:,2).^2);
normFirm = sqrt(FirmDistnace(:,1).^2+FirmDistnace(:,2).^2);
figure
plot(normVRep)
hold on
plot(normFirm,'r')
% daspect([1 1 1])
grid on
legend('vRep','FIRM')
xlabel('simulation step')
ylabel('Distance travelled b/w two steps *(m)')
title('comparison of v-rep and FIRM')
figure
plot(traj.u(1,:),'b')
xlabel('simulation step')
ylabel('FIRM generated linear velocity (m/s)')
grid on
% daspect([1 1 1])

figure
plot(V_rep(:,1),V_rep(:,2), 'b')
hold on
plot(Firm(:,1),Firm(:,2),'r')
legend('V-rep (Dynamic)','FIRM (Kinematic')
xlabel('x-coordinate of robot')
ylabel('y-coordinate of robot')
title('Trajectory')
grid on
axis equal