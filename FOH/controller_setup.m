%%set up the controller
%% intial states and input
x0 = [0 0 0 0]';
u0 = [0 0]';
%% Discretize the continuous model using ZOH method in LateralDynamicsDT
ks = 0;     %curveture
Vx = 15;    %longitudinal velocity
Ts = 0.1;
PredictionHorizon = 100;
[Ad,Bd,Cd,Dd,U,Y,X,DX,DK] = LateralDynamicsDT(Vx,ks,x0,u0,Ts);
dsys = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
dsys.InputName = {'Fyf','Mx','md1','md2','md3','md4'};
dsys.StateName = {'v','r','Theta','Y'};
dsys.OutputName = dsys.StateName;
plant = setmpcsignals(dsys,'MD',[3 4 5 6]);
%% design MPC controller at the Nomial Operating point
status = mpcverbosity('off');
mpc007 = mpc(plant);

mpc007.PredictionHorizon = 100;%100;
mpc007.ControlHorizon = 3;%3;

mpc007.ManipulatedVariables(1).Min = -10000;%5000
mpc007.ManipulatedVariables(1).Max = 10000;

mpc007.ManipulatedVariables(2).Min = -1000;%1500
mpc007.ManipulatedVariables(2).Max = 1000;

% mpc007.ManipulatedVariables(1).RateMin = -1000*Ts;
% mpc007.ManipulatedVariables(1).RateMax = 1000*Ts;

mpc007.ManipulatedVariables(2).RateMin = -1000*Ts;
mpc007.ManipulatedVariables(2).RateMax = 1000*Ts;

% mpc007.OutputVariables(1).MinECR = 10;
% mpc007.OutputVariables(1).MaxECR = 10;
mpc007.OutputVariables(2).MinECR = 5.74;
mpc007.OutputVariables(2).MaxECR = 5.74;
mpc007.OutputVariables(3).MinECR = 5.74;
mpc007.OutputVariables(3).MaxECR = 5.74;
mpc007.OutputVariables(4).MinECR = 1.41;
mpc007.OutputVariables(4).MaxECR = 1.41;

% Standard Cost Function
% mpc007.Weights.ManipulatedVariables  = [0 4e-4];%it is 4e-5 in the paper but it oscilates
% mpc007.Weights.ManipulatedVariablesRate = [3e-5 8e-5];%3e-5 8e-5
% mpc007.Weights.OutputVariables = [0 0 200 4];

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

%% specify Mixed I/O constraints
rmax = g*miu/Vx; %max yaw rate
threshold = atan(3*miu*Fz/C_a);
r = x0(2);
vmax = lr*r+Vx*threshold;   %max lateral velocity
vmin = lr*r-Vx*threshold;   %min lateral velocity
mpc007.OutputVariables(1).Min = vmin;
mpc007.OutputVariables(1).Max = vmax;
mpc007.OutputVariables(2).Min = -rmax;
mpc007.OutputVariables(2).Max = rmax;
mpc007.OutputVariables(3).Min = -pi/2;
mpc007.OutputVariables(3).Max = pi/2;
mpc007.OutputVariables(4).Min = -10;
mpc007.OutputVariables(4).Max = 10;

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