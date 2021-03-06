clc; clear all; close all; vrclose all;
world = vrworld('Arm_3Joints.wrl');
open(world);
view(world); vrdrawnow;reload(world);

x=-10;
y=25;
z=10;

Zo=2.3;
desiredPos=[x,y,z-Zo];
L1 = 13.6; %cm
L2 = 6.3; %cm
L3 = 14.7; %cm

world.J2.translation=[0 L1 0];
world.arm1.scale = [1 L1 1];
world.J3.translation=[0 L2 0];
world.arm2.scale = [1 L2 1];
world.lapiz.translation=[0 L3 0];
world.arm3.scale = [1 L3 1];
world.ref.translation=desiredPos;

[j1Rot,j2Rot,j3Rot]=initialGuess3(x,y,z,world)
[x,y,z] = forwardKin3(j1Rot,j2Rot,j3Rot)

%%%% This below is not needed now because we are displaying in real-time the iterations

% sameTime=0; % 0 -> J2 starts moving after J1 finishes moving, 1 -> J2 and J1 moves at the same time.
% drawTime = 0.01; % lower the value for faster the visualization (seconds)
% j1step=sign(j1Rot);
% j2step=sign(j2Rot);
% j3step=sign(j3Rot);
% 
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
%     for j3Ang=0:j3step:j3Rot
%         world.J3.rotation = [1 0 0 j3Ang*3.1416/180];
%         vrdrawnow;
%         pause(drawTime);
%     end
% elseif (sameTime==1)
%     magj1Rot = abs(j1Rot);
%     magj2Rot = abs(j2Rot);
%     magj3Rot = abs(j3Rot);
%     maxSteps = max([magj1Rot magj2Rot magj3Rot]);
%     for step=0:maxSteps;
%         if (magj1Rot >= step)
%             world.J1.rotation = [0 0 1 step*j1step*3.1416/180];
%             vrdrawnow;
%         end
%         if (magj2Rot >= step)
%             world.J2.rotation = [0 0 1 step*j2step*3.1416/180];
%             vrdrawnow;
%         end
%         if (magj3Rot >= step)
%             world.J3.rotation = [1 0 0 step*j3step*3.1416/180];
%             vrdrawnow;
%         end
%         pause(drawTime);
%     end
% end