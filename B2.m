clear;clc;
dt = 0.01;
T = 18;
b = 1.4;
c = 1;
ut = 1.5;
n = T / dt;

a_theory = 8;
a_g = a_theory + (0.15)*(2 * randi(2) - 3)*a_theory;

X = [0;0;a_g];

l = linspace(0,T, n);
u = 5*square(l/1.5*pi);

P = zeros(3,3);
P(2,2) = (2*pi/180)^2;
P(1,1) = (110*pi/180)^2;
P(3,3) = 0.1^2;
StdNoise = (5*pi/180)^2;


for i = 1:n
    x1 = X(1); x2 = X(2); a = X(3);
    
    J = [1 dt 0;-dt*a*cos(x1) 1-b*dt -x2*dt;0 0 1];
    H = [1 0 0];
    % R = [(110*pi/180)^2 0; 0 (2*pi/180)^2];
    R = (5*pi/180)^2;
    Pu = (0.05)^2;
    Ju = [0;0; dt*c];

    Qu = Ju*Pu*transpose(Ju);
    P = J*P*transpose(J) + Qu;
    C = P*H';
    S = H*P*H'+R;
    K = C/S;
    noise = randn(1,1)*StdNoise; % Sample of WGN, having standard deviation StdNoise
    X_ = [x1+dt*x2;x2+ dt*(-a*sin(x1) -b*x2 + 2*u(i)); a];
    sim =  X_ + noise;
    sim(3) = 8;
    dif = sim - X_;
    X = X_ + K.*dif;
end

