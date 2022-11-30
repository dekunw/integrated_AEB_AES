%%set up the controller
%% intial states and input
x0 = [0 0 0 0]';
u0 = [0 0]';
%% Discretize the continuous model using ZOH method in LateralDynamicsDT
ks = 0;     %curveture
Vx = 15;    %longitudinal velocity
Ts = 0.05;
PredictionHorizon = 60;
[Ad,Bd,Cd,Dd,U,Y,X,DX,DK] = LateralDynamicsDT(Vx,ks,x0,u0,Ts);
dsys = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
dsys.InputName = {'Fyf','Mx','md1','md2','md3','md4'};
dsys.StateName = {'v','r','Theta','Y'};
dsys.OutputName = dsys.StateName;
plant = setmpcsignals(dsys,'MD',[3 4 5 6]);
%% design MPC controller at the Nomial Operating point
status = mpcverbosity('off');
mpc007 = mpc(plant);

mpc007.PredictionHorizon = PredictionHorizon;%100;
mpc007.ControlHorizon = 1;%3;

mpc007.ManipulatedVariables(1).Min = -10000;%5000
mpc007.ManipulatedVariables(1).Max = 10000;

mpc007.ManipulatedVariables(2).Min = -1000;%1500
mpc007.ManipulatedVariables(2).Max = 1000;

% mpc007.ManipulatedVariables(1).RateMin = -3000*Ts;
% mpc007.ManipulatedVariables(1).RateMax = 3000*Ts;
% mpc007.ManipulatedVariables(2).RateMin = -1000*Ts;
% mpc007.ManipulatedVariables(2).RateMax = 1000*Ts;

%Quadratic Cost Function
mpc007.Weights.MV  = {[0 0;0 4e-5]};%it is 4e-5 in the paper but it oscilates
mpc007.Weights.MVRate = {[3e-5 0;0 8e-5]};%3e-5 8e-5
mpc007.Weights.OutputVariables = {[0 0 0 0;0 0 0 0;0 0 200 0;0 0 0 4]};

mpc007.Model.Nominal = struct('U',U,'Y',Y,'X',X,'DX',DX);

%% specify parameters of the vehicle
miu = 0.7; %frction cooeficient 0.7 for dry road and 0.4 for wet
g = 9.81; % gravity
m = 2272;   %mass
lf = 1.11;  % distance from front axle to the mass center
lr = 1.67;  % distance from rear axle to the mass center
C_a = 1.3e5; % rear cornering stifness
Fz = m*g*lf/(lf+lr); % the load that beared by the rear tire
CarWidth =1.4;

%% specify Mixed I/O constraints
rmax = g*miu/Vx; %max yaw rate
threshold = atan(3*miu*Fz/C_a);
r = x0(2);
vmax = lr*r+Vx*threshold;   %max lateral velocity
vmin = lr*r-Vx*threshold;   %min lateral velocity
mmm = linspace(0,5,50)';
mpc007.OutputVariables(1).Min = vmin;
mpc007.OutputVariables(1).Max = vmax;
mpc007.OutputVariables(2).Min = -rmax;
mpc007.OutputVariables(2).Max = rmax;
mpc007.OutputVariables(3).Min = -pi/2;
mpc007.OutputVariables(3).Max = pi/2;
mpc007.OutputVariables(4).Min = -10;
mpc007.OutputVariables(4).Max = 10;
