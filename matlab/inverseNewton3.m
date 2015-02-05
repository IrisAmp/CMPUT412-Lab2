function [a1,a2,a3] = inverseNewton3(posx,posy,posz,a1,a2,a3)

dx=[a1;a2;a3];
y=[posx;posy;posz];
for n=0:100
    a1=dx(1);
    a2=dx(2);
    a3=dx(3);
    [cor1, cor2, cor3]=forwardKin3(a1,a2,a3);
    [cor1_1, cor2_1, cor3_1]=forwardKin3(a1+0.001,a2,a3);
    [cor1_2, cor2_2, cor3_2]=forwardKin3(a1,a2+0.001,a3);
    [cor1_3, cor2_3, cor3_3]=forwardKin3(a1,a2,a3+0.001);
    f = [cor1;cor2;cor3];
    df = [  cor1_1-cor1 cor1_2-cor1 cor1_3-cor1;
            cor2_1-cor2 cor2_2-cor2 cor2_3-cor2;
            cor3_1-cor3 cor3_2-cor3 cor3_3-cor3];
    dx=dx+inv(df)*(y-f)/1000;
end
a1 = dx(1);
a2 = dx(2);
a3 = dx(3);