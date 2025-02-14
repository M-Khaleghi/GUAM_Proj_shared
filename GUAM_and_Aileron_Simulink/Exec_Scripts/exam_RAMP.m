%% sim parameters
%model = 'sm_aileron_actuator';
%projectPath = 'D:/Aircraft_project/Aileron_Actuator.prj'; % Specify the path to your Simulink project
project = simulinkproject(projectPath); % Open the specified Simulink project
model = 'GUAM';
%model2 = fullfile(project.RootFolder,'sm_aileron_actuator.slx'); % Specify the path to your Simulink model within the project

% use timeseries input
userStruct.variants.refInputType = 1; % 1=FOUR_RAMP, 2= ONE_RAMP, 3=Timeseries, 4=Default(doublets)

%% Define the target structure and provide the Ramp settings
target = struct('tas', 90, 'gndtrack', 0,'stopTime', 30);

% Initialize the sim
simSetup;

% Specify the Ramp settings
SimPar_Set_Ramps; % Sets a basic ramp program
% SimPar_Null_Ramps; % Nulls out the ramps (aircraft stays trimmed)

% Open the simulation
open(model);
%open(model2);