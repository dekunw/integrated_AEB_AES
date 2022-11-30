% Set up Script for the Lane Following Example
%
% This script initializes the lane following example model. It loads
% necessary control constants and sets up the buses required for the
% referenced model.
%
%   This is a helper script for example purposes and may be removed or
%   modified in the future.

%   Copyright 2018 The MathWorks, Inc.

%% General Model Parameters
Ts = 0.1;               % Simulation sample time  (s)

%% Path following Controller Parameters
time_gap        = 1.5;   % time gap               (s)
default_spacing = 10;    % default spacing        (m)
max_ac          = 2;     % Maximum acceleration   (m/s^2)
min_ac          = -3;    % Minimum acceleration   (m/s^2)
max_steer       = 0.26;  % Maximum steering       (rad)
min_steer       = -0.26; % Minimum steering       (rad) 
PredictionHorizon = 30;  % Prediction horizon     

%% Create driving scenario
% The scenario name is a scenario file created by the Driving Scenario Designer App. 
scenariosNames = {
    'ACC_01_ISO_TargetDiscriminationTest.mat',...          % scenarioId = 1
    'ACC_02_ISO_AutoRetargetTest.mat',...                  % scenarioId = 2
    'ACC_03_ISO_CurveTest.mat',...                         % scenarioId = 3
    'ACC_04_StopnGo.mat',...                               % scenarioId = 4
    'LFACC_01_DoubleCurve_DecelTarget.mat',...             % scenarioId = 5
    'LFACC_02_DoubleCurve_AutoRetarget.mat',...            % scenarioId = 6
    'LFACC_03_DoubleCurve_StopnGo.mat',...                 % scenarioId = 7
    'LFACC_04_Curve_CutInOut.mat',...                      % scenarioId = 8
    'LFACC_05_Curve_CutInOut_TooClose.mat',...             % scenarioId = 9
    'AEB_CCRb_6_initialGap_40m_stop_inf.mat',...           % scenarioId = 10
    'AEB_PedestrianChild_Nearside_50width_overrun.mat' ,...     % scenarioId = 11
    'AEB_wdk.mat'                                              % scenarioId = 12 // only steering works
    };
scenarioStopTimes = [19.82 17.99 21.99 28.61 27.3 39.66 34.28 22.8 23.47 20 25 10];
scenarioId = 12;

% The scenario file is converted to a drivingScenario object
% initial conditions of ego car and actor profiles
[scenario,egoCar,actor_Profiles] = helperSessionToScenario(scenariosNames{scenarioId});

if scenarioId == 8 || scenarioId == 9
    v_set = 21.5;       % ACC set speed (m/s)
elseif scenarioId == 10
    v_set = 15;
else
    v_set = egoCar.v0;  % ACC set speed (m/s)
end

% Initial condition for the ego car in ISO 8855 coordinates
v0_ego   = egoCar.v0;          % Initial speed of the ego car           (m/s)
x0_ego   = egoCar.x0;          % Initial x position of ego car          (m)
y0_ego   = egoCar.y0;          % Initial y position of ego car          (m)
yaw0_ego = egoCar.yaw0;        % Initial yaw angle of ego car           (rad)

% Convert ISO 8855 to SAE J670E coordinates
y0_ego = -y0_ego;
yaw0_ego = -yaw0_ego;

% Define a simulation stop time
simStopTime = scenarioStopTimes(scenarioId);

%% Tracking and Sensor Fusion Parameters                        Units
clusterSize = 4;        % Distance for clustering               (m)
assigThresh = 20;       % Tracker assignment threshold          (N/A)
M           = 2;        % Tracker M value for M-out-of-N logic  (N/A)
N           = 3;        % Tracker M value for M-out-of-N logic  (N/A)
numCoasts   = 5;        % Number of track coasting steps        (N/A)
numTracks   = 100;       % Maximum number of tracks              (N/A)
numSensors  = 4;        % Maximum number of sensors             (N/A)

% Position and velocity selectors from track state
% The filter initialization function used in this example is initcvekf that 
% defines a state that is: [x;vx;y;vy;z;vz]. 
posSelector = [1,0,0,0,0,0; 0,0,1,0,0,0]; % Position selector   (N/A)
velSelector = [0,1,0,0,0,0; 0,0,0,1,0,0]; % Velocity selector   (N/A)

%% Ego Car Parameters
% Dynamics modeling parameters
miu = 1;          % Friction coefficient
g = 9.81;          % gravity
m       = 2272;     % Total mass of vehicle                          (kg)
Iz      = 4600;     % Yaw moment of inertia of vehicle               (m*N*s^2)
lf      = 1.11;      % Longitudinal distance from c.g. to front tires (m)
lr      = 1.67;      % Longitudinal distance from c.g. to rear tires  (m)
Cf      = 1.3e5;    % C_ar in dynamics is 1.3e5 Cornering stiffness of front tires             (N/rad)
Cr      = 1.3e5;    % C_ar in dynamics is 1.3e5 Cornering stiffness of rear tires              (N/rad)
tau     = 0.5;      % time constant for longitudinal dynamics 1/s/(tau*s+1)
%% Bus Creation
% Load the Simulink model
modelName = 'Integrated_controlled_system';
wasModelLoaded = bdIsLoaded(modelName);
if ~wasModelLoaded
    load_system(modelName)
end

% Create buses for lane sensor and lane sensor boundaries
createLaneSensorBuses;

% load the bus for scenario reader
blk=find_system(modelName,'System','driving.scenario.internal.ScenarioReader');
s = get_param(blk{1},'PortHandles');
get(s.Outport(1),'SignalHierarchy');

% Set the scenario reader file name to the selected scenario
set_param(blk{1},'ScenarioFileName',scenariosNames{scenarioId});

% Create the bus of tracks (output from referenced model)
refModel = 'Controller';
wasReModelLoaded = bdIsLoaded(refModel);
if ~wasReModelLoaded
    load_system(refModel)
    blk=find_system(refModel,'System','multiObjectTracker');
    multiObjectTracker.createBus(blk{1});
    close_system(refModel)
else
    blk=find_system(refModel,'System','multiObjectTracker');
    multiObjectTracker.createBus(blk{1});
end

%% Code generation
% To generate code, uncomment the following commands.
% refModel = 'LFRefMdl';
% rtwbuild(refModel)

%% AEB controller parameters
headwayOffset=3.7;% headway offset                         (m)
PB1_decel=3.8;%1st stage Partial Braking deceleration (m/s^2)
PB2_decel=5.3;% 2nd stage Partial Braking deceleration (m/s^2)
FB_decel=9.8;% Full Braking deceleration              (m/s^2)
timeToReact=1.2;% driver reaction time                   (sec)
timeMargin=0;
driver_decel=4.0;% driver braking deceleration            (m/s^2)

max_dc=-10; % Maximum deceleration   (m/s^2)
% max_ac=2;    % Maximum acceleration   (m/s^2)


%% Lateral controller set-up
x0 = [0 0 0 0]';
u0 = [0 0]';
ks = 0;
Ts1 = 0.1;
%%Discretize the continuous model using ZOH method in LateralDynamicsDT
PredictionHorizon1 = 60;
[Ad1,Bd1,Cd1,Dd1,U1,Y1,X1,DX1,DK1] = LateralDynamicsDT05(v_set,ks,x0,u0,Ts1);
dsys1 = ss(Ad1,Bd1,Cd1,Dd1,'Ts',Ts);
dsys1.InputName = {'Fyf','Mx','md1','md2','md3','md4'};
dsys1.StateName = {'v','r','Theta','Y'};
dsys1.OutputName = dsys1.StateName;
plant1 = setmpcsignals(dsys1,'MD',[3 4 5 6]);
%%design MPC controller at the Nomial Operating point
status = mpcverbosity('off');
mpc008 = mpc(plant1);

mpc008.PredictionHorizon = PredictionHorizon1;%100;
mpc008.ControlHorizon = 1;%3;

mpc008.ManipulatedVariables(1).Min = -10000;%5000
mpc008.ManipulatedVariables(1).Max = 10000;

mpc008.ManipulatedVariables(2).Min = -1000;%1500
mpc008.ManipulatedVariables(2).Max = 1000;

% mpc007.ManipulatedVariables(1).RateMin = -1000*Ts;
% mpc007.ManipulatedVariables(1).RateMax = 1000*Ts;
% mpc007.ManipulatedVariables(2).RateMin = -1000*Ts;
% mpc007.ManipulatedVariables(2).RateMax = 1000*Ts;

%Quadratic Cost Function
mpc008.Weights.MV  = {[0 0;0 4e-5]};%it is 4e-5 in the paper but it oscilates
mpc008.Weights.MVRate = {[3e-5 0;0 8e-5]};%3e-5 8e-5
mpc008.Weights.OutputVariables = {[0 0 0 0;0 0 0 0;0 0 200 0;0 0 0 4]};
mpc008.Model.Nominal = struct('U',U1,'Y',Y1,'X',X1,'DX',DX1);

%%specify parameters of the vehicle
Fz = m*g*lf/(lf+lr)/2; % the load that beared by the rear tire

%%specify Mixed I/O constraints
rmax = g*miu/v_set; %max yaw rate
threshold = atan(3*miu*Fz/Cr);
r = x0(2);
vmax = lr*r+v_set*threshold;   %max lateral velocity
vmin = lr*r-v_set*threshold;   %min lateral velocity
mpc008.OutputVariables(1).Min = vmin;
mpc008.OutputVariables(1).Max = vmax;
mpc008.OutputVariables(2).Min = -rmax;
mpc008.OutputVariables(2).Max = rmax;
mpc008.OutputVariables(3).Min = -pi/2;
mpc008.OutputVariables(3).Max = pi/2;
mpc008.OutputVariables(4).Min = -10;
mpc008.OutputVariables(4).Max = 10;
load('Fyf_paras.mat');
1   %see if the whole script is loaded