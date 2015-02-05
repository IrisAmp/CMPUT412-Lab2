clc; clear all; close all; vrclose all;
world = vrworld('Arm_2Joints.wrl');
open(world);
view(world); vrdrawnow;reload(world);

x=-10;
y=15;

L1 = 13.5; %cm
L2 = 14.2; %cm
world.arm1.scale = [1 L1 1];
world.arm2.scale = [1 L2 1];
world.J2.translation=[0 L1 0];
world.lapiz.translation=[0 L2 0];
world.ref.translation = [x y 0];

[rot1,rot2]=initialGuess(x,y,world)
[x,y]=forwardKin(rot1,rot2)


%%%% This below is not needed now because we are displaying in real-time the iterations
% j1Rot=rot1;
% j2Rot=rot2;
% sameTime=1; % 0 -> J2 starts moving after J1 finishes moving, 1 -> J2 and J1 moves at the same time.
% drawTime = 0.01; % lower the value for faster the visualization (seconds)
% j1step=sign(j1Rot);
% j2step=sign(j2Rot);
% if (sameTime==0)
%     for j1Ang=0:j1step:j1Rot
%         world.J1.rotation = [0 0 1 j1Ang*3.1416/180];
%         vrdrawnow;
%         pause(drawTime);
%     end
%     for j2Ang=0:j2step:j2Rot
%         world.J2.rotation = [0 0 1 j2Ang*3.1416/180];
%         vrdrawnow;
%         pause(drawTime);
%     end
% elseif (sameTime==1)
%     magj1Rot = abs(j1Rot);
%     magj2Rot = abs(j2Rot);
%     maxSteps = max([magj1Rot magj2Rot]);
%     for step=0:maxSteps;
%         if (magj1Rot >= step)
%             world.J1.rotation = [0 0 1 step*j1step*3.1416/180];
%             vrdrawnow;
%         end
%         if (magj2Rot >= step)
%             world.J2.rotation = [0 0 1 step*j2step*3.1416/180];
%             vrdrawnow;
%         end
%         pause(drawTime);
%     end
% end
% 
% vrdrawnow;
% %close(world);