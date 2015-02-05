function [x,y,z] = forwardKin3(a1,a2,a3)
L1 = 13.6; %cm
L2 = 6.3; %cm
L3 = 14.7; %cm
Zo=2.3;
a1=a1*pi/180;
a2=a2*pi/180;
a3=a3*pi/180;

x = (L1 * cos(a1)) + (L2 * cos(a1 + a2)) + (L3*cos(a1+a2))*cos(a3);
y = (L1 * sin(a1)) + (L2 * sin(a1 + a2)) + (L3*sin(a1+a2))*cos(a3);
z = Zo + L3*sin(a3);