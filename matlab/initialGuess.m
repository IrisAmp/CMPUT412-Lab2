function [a1,a2] = initialGuess(posx,posy,world)

a1=1;
a2=1;
[x, y]=forwardKin(a1,a2);

N=100;
difX=(posx-x)/N;
difY=(posy-y)/N;

for n=1:N
    [an1,an2]=inverseNewto(difX*n+x,difY*n+y,a1,a2);
    a1=an1;a2=an2;
    world.J1.rotation = [0 0 1 a1*3.1416/180];
    world.J2.rotation = [0 0 1 a2*3.1416/180];
    vrdrawnow;
    pause(0.01);
end