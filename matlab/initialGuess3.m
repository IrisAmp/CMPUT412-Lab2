function [a1,a2,a3] = initialGuess3(posx,posy,posz,world)

a1=1;
a2=1;
a3=1;
[x, y, z]=forwardKin3(a1,a2,a3);

N=100;
difX=(posx-x)/N;
difY=(posy-y)/N;
difZ=(posz-z)/N;

for n=1:N
    [an1,an2,an3]=inverseNewton3(difX*n+x,difY*n+y,difZ*n+z,a1,a2,a3);
    a1=an1;a2=an2;a3=an3;
    world.J1.rotation = [0 0 1 a1*3.1416/180];
    world.J2.rotation = [0 0 1 a2*3.1416/180];
    world.J3.rotation = [1 0 0 a3*3.1416/180];
    vrdrawnow;
    pause(0.01);
end