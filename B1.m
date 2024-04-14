%% Part B1
clear;clc;
dt = 0.01;
T = 18;
a = 8;
b = 1.4;
c = 1;
ut = 1.5;
n = T / dt;

X = [0;0];

l = linspace(0,T, n);
u = 5*square(l/1.5*pi);

P = zeros(3,3);
P(3,3) = 0.1^2;

for i = 1:n
    x1 = X(1); x2 = X(2);
    
    
    J = [1 dt 0;-dt*a*cos(x1) 1-b*dt -x2*dt;0 0 1];
    H = [1 0 0];
    R = [(110*pi/180)^2 0; 0 (2*pi/180)^2];

    Pu = (5*pi/180)^2;
    Ju = [0;0; dt*c];

    Qu = Ju*Pu*transpose(Ju);
    P = J*P*transpose(J) + Qu;

    X = [x1+dt*x2;x2+ dt*(-a*sin(x1) -b*x2 + 2*u(i))];
end
