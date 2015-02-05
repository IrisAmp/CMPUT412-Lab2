function [x,y] = forwardKin(a1,a2)
L1 = 13.5;
L2 = 14.2;
a1=a1*pi/180;
a2=a2*pi/180;
x = (L1 * cos(a1)) + (L2 * cos(a1 + a2));
y = (L1 * sin(a1)) + (L2 * sin(a1 + a2));