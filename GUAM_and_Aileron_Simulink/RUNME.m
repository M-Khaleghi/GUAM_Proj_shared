% This script is a toplevel script that executes the users desired example case:
Ts = 0.05
load("RNN-Model-GUAM-DES-diag-v01")
addpath('./Exec_Scripts/');
u_choice = input(sprintf('Specify the desired example case to run:\n\t(1) Sinusoidal Timeseries\n\t(2) Hover to Transition Timeseries\n\t(3) Cruise Climbing Turn Timeseries\n\t(4) Ramp demo\n'));

switch u_choice
    case 1
        if exist('userStruct') && userStruct.variants.fmType == ForceMomentEnum.Polynomial
            msgbox('Polynomial Model won''t work for this case (forward speed is below range which polynomial model covers)');
            return;
        end
        exam_TS_Sinusoidal_traj;
    case 2
        if exist('userStruct') && userStruct.variants.fmType == ForceMomentEnum.Polynomial
            msgbox('Polynomial Model won''t work for this case (forward speed is below range which polynomial model covers)');
            return;
        end
        exam_TS_Hover2Cruise_traj3
    case 3
        exam_TS_Cruise_Climb_Turn_traj
    case 4
        msgbox('simPlots_GUAM.m script is only set to work with trajectories (not all plots will be created).');
        exam_RAMP
    otherwise
        fprintf('User needs to supply selection choice (1-4)\n')
        return3
end

% Execute the model
sim(model);
% Create sample output plots
simPlots_GUAM;