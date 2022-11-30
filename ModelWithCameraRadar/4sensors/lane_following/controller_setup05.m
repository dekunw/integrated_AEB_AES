%%set up the controller
%% intial states and input
x0 = [0 0 0 0]';
u0 = [0 0]';
%% Discretize the continuous model using ZOH method in LateralDynamicsDT
ks = 0;     %curveture
Vx = 15;    %longitudinal velocity
Ts = 0.1;
PredictionHorizon = 100;
[Ad1,Bd1,Cd1,Dd1,U1,Y1,X1,DX1,DK1] = LateralDynamicsDT05(Vx,ks,x0,u0,Ts);
dsys1 = ss(Ad1,Bd1,Cd1,Dd1,'Ts',Ts);
dsys1.InputName = {'Fyf','Mx','md1','md2','md3','md4'};
dsys1.StateName = {'v','r','Theta','Y'};
dsys1.OutputName = dsys1.StateName;
plant1 = setmpcsignals(dsys1,'MD',[3 4 5 6]);
%% design MPC controller at the Nomial Operating point
status = mpcverbosity('off');
mpc008 = mpc(plant1);

mpc008.PredictionHorizon = 100;%100;
mpc008.ControlHorizon = 3;%3;

mpc008.ManipulatedVariables(1).Min = -10000;%5000
mpc008.ManipulatedVariables(1).Max = 10000;

mpc008.ManipulatedVariables(2).Min = -1000;%1500
mpc008.ManipulatedVariables(2).Max = 1000;

% mpc007.ManipulatedVariables(1).RateMin = -1000*Ts;
% mpc007.ManipulatedVariables(1).RateMax = 1000*Ts;
% mpc007.ManipulatedVariables(2).RateMin = -1000*Ts;
% mpc007.ManipulatedVariables(2).RateMax = 1000*Ts;

%Standard Cost Function
% mpc007.Weights.ManipulatedVariables  = [0 4e-4];%it is 4e-5 in the paper but it oscilates
% mpc007.Weights.ManipulatedVariablesRate = [3e-5 8e-5];%3e-5 8e-5
% mpc007.Weights.OutputVariables = [0 0 200 4];

%Quadratic Cost Function
mpc008.Weights.MV  = {[2e-4 0;0 8e-4]};%it is 4e-5 in the paper but it oscilates
mpc008.Weights.MVRate = {[4e-4 0;0 16e-4]};%3e-5 8e-5
mpc008.Weights.OutputVariables = {[0 0 0 0;0 0 0 0;0 0 200 0;0 0 0 4]};

mpc008.Model.Nominal = struct('U',U1,'Y',Y1,'X',X1,'DX',DX1);

%% specify parameters of the vehicle
miu = 0.7; %frction cooeficient 0.7 for dry road and 0.4 for wet
g = 9.81; % gravity
m = 2272;   %mass
lf = 1.11;  % distance from front axle to the mass center
lr = 1.67;  % distance from rear axle to the mass center
C_a = 1.3e5; % rear cornering stifness
Fz = m*g*lf/(lf+lr)/2; % the load that beared by the rear tire

%% specify Mixed I/O constraints
rmax = g*miu/Vx; %max yaw rate
threshold = atan(3*miu*Fz/C_a);
r = x0(2);
vmax = lr*r+Vx*threshold;   %max lateral velocity
vmin = lr*r-Vx*threshold;   %min lateral velocity
mpc008.OutputVariables(1).Min = vmin;
mpc008.OutputVariables(1).Max = vmax;
mpc008.OutputVariables(2).Min = -rmax;
mpc008.OutputVariables(2).Max = rmax;
mpc008.OutputVariables(3).Min = -pi/2;
mpc008.OutputVariables(3).Max = pi/2;
mpc008.OutputVariables(4).Min = -10;
mpc008.OutputVariables(4).Max = 10;

%% from here to the end to be commented
% % F=[1 -lr 0 0;-1 lr 0 0;0 1 0 0;0 -1 0 0;0 0 0 1;0 0 0 -1];
% % E=zeros(6,2);
% % G=[Vx*threshold Vx*threshold rmax rmax 10 10]';
% % S=zeros(6,4);
% F=[0 0 0 1];
% E=zeros(1,2);
% G=-1;
% S=zeros(1,4);
% 
% setconstraint(mpc007,E,F,G,[]',S);
% 
% %%
% % Use a constant reference signal.
% refSignal = [0 0 0 0.1];
% 
% %% Initialize plant and controller states
% x = x0;
% u = u0;
% egoStates = mpcstate(mpc007);
% 
% % The simulation time is |10| seconds.
% T = 0:Ts:10;
% 
% %%
% % Log simulation data for plotting.
% % saveSlope = zeros(length(T),1);
% % saveIntercept = zeros(length(T),1);
% ympc = zeros(length(T),size(Cd,1));
% umpc = zeros(length(T),size(Bd,2));
% 
% %%
% % Run the simulation.
% for k = 1:length(T)
%     % Obtain new plant model and output measurements for interval |k|.
%     [Ad,Bd,Cd,Dd,U,Y,X,DX,DK] = LateralDynamicsDT(Vx,ks,x,u,Ts);
%     measurements = Cd * x + Dd * [u;zeros(4,1)];
%     ympc(k,:) = measurements';
%     
%     % Determine whether the vehicle sees the obstacle, and update the mixed
%     % I/O constraints when obstacle is detected.
%     if k<12
%         F=[0 0 0 1];
%         E=zeros(1,2);
%         G=1;
%         S=zeros(1,4);
%     else
%         F=[0 0 0 1];
%         E=zeros(1,2);
%         G=-4;
%         S=zeros(1,4);
%     end
%    
%     % Prepare new plant model and nominal conditions for adaptive MPC.
%     newPlant = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
%     newPlant = setmpcsignals(newPlant,'MD',[3 4 5 6]);
%     newNominal = struct('U',U,'Y',Y,'X',X,'DX',DX);
%     
%     % Prepare new mixed I/O constraints.
%     options = mpcmoveopt;
%     options.CustomConstraint = struct('E',E,'F',F,'G',G,'S',S);
%     
%     % Compute optimal moves using the updated plant, nominal conditions,
%     % and constraints.
%     [u,Info] = mpcmoveAdaptive(mpc007,egoStates,newPlant,newNominal,...
%         measurements,refSignal,[],options);
%     umpc(k,:) = [u;DK']';
%     
%     % Update the plant state for the next iteration |k+1|.
%     x = Ad * x + Bd * [u;DK'];
% end
% 
% mpcverbosity(status);