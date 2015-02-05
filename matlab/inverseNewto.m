function [a1,a2] = inverseNewto(posx,posy,a1,a2)

dx=[a1;a2];
y=[posx;posy];
for n=0:100
    a1=dx(1);
    a2=dx(2);
    [cor1, cor2]=forwardKin(a1,a2);
    [cor1_1, cor2_1]=forwardKin(a1+0.001,a2);
    [cor1_2, cor2_2]=forwardKin(a1,a2+0.001);
    f = [cor1;cor2];
    df = [  cor1_1-cor1 cor1_2-cor1;
            cor2_1-cor2 cor2_2-cor2];
    dx=dx+inv(df)*(y-f)/1000;
end
a1 = dx(1);
a2 = dx(2);