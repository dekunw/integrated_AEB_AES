function [A, B, C, D, U, Y, X, DX, DK] = LateralDynamicsDT(Vx,ks,x,u,Ts)

%Plant Model parameters
m = 2272;   %mass
Iz = 4600;  %rotational inertia
lf = 1.11;  % distance from front axle to the mass center
lr = 1.67;  % distance from rear axle to the mass center
C_a = 1.822e5; % rear cornering stiffness 

%Enviroment parameters
miu = 0.7; %frction cooeficient 0.7 for dry road and 0.4 for wet
g = 9.81; % gravity

Fz = m*g*lf/(lf+lr); % the load that beared by the rear tire

%calculate alpha
Vy = x(1); %lateral velocity
r = x(2);  %yaw rate
alphar_ = Vy/Vx-lr*r/Vx;   %calculate linearzing point

%tire model
threshold = atan(3*miu*Fz/C_a);     % threshold for the tire linear region
if abs(alphar_)<threshold
    %the rear lateral force value (f(x0)) at the linearzing point
    Fyr_ = -C_a*tan(alphar_)+C_a^2/(3*miu*Fz)*abs(tan(alphar_))*tan(alphar_)-C_a^3/(27*miu^2*Fz^2)*tan(alphar_)^3;
    
    %the slope of linearized model (f‘(x0))
    if alphar_>= 0
        Cr_ = -C_a*sec(alphar_)^2+C_a^2/(3*miu*Fz)*(2*sec(alphar_)^2*tan(alphar_))-C_a^3/(27*miu^2*Fz^2)*(3*tan(alphar_)^2*sec(alphar_)^2);
    else
        Cr_ = -C_a*sec(alphar_)^2-C_a^2/(3*miu*Fz)*(2*sec(alphar_)^2*tan(alphar_))-C_a^3/(27*miu^2*Fz^2)*(3*tan(alphar_)^2*sec(alphar_)^2);
    end
else
    
    %the saturation region of tire model
    Fyr_ = -miu*Fz*sign(alphar_);
    Cr_ = 0;
end

%Continuous-time model
Ac = [Cr_/(m*Vx) -lr*Cr_/(m*Vx)-Vx 0 0;
    -lr*Cr_/(Iz*Vx) lr^2*Cr_/(Iz*Vx) 0 0;
    0 1 0 0;
    1 0 Vx 0];
Bc = [1/m 0;
    lf/Iz 1/Iz;
    0 0;
    0 0];
dcc = [(Fyr_-Cr_*alphar_)/m;
    -lr*(Fyr_-Cr_*alphar_)/Iz;
    -Vx*ks;
    0];

Cc =eye(4);
Dc =zeros(4,2);

% M = expm([[Ac Bc dcc]*Ts;zeros(3,7)]);
% A = M(1:4,1:4);
% B = M(1:4,5:6);
% DK = M(1:4,7);
% C = Cc;
% D = Dc;
% U = u;
% Y = x;
% X = x;
% DX = A*x+B*u-x;

Aca = Ac;
Bca = [Bc eye(4)];
M = expm([[Aca Bca zeros(4,1)]*Ts;zeros(7,11)]);
A = M(1:4,1:4);    %the discrete matrix A for augmented model
B = M(1:4,5:10);   %the discrete matrix B for augmented model
%store the state space matrix for ct time step
C = Cc;
D = [Dc,zeros(4,4)];  % always be zeros(4,6) in fact
U = [u;dcc];          % measured disturbance dcc is input as well
Y = x;               % output is all of the four states
X = x;               % four states
DX = A*x+B*[u;dcc]-x;%the states increment
DK = dcc';   