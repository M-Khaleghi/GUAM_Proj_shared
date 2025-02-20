function solverBlock_pth = sm_aileron_actuator_setsolver(mdl,deskreal)
% Copyright 2011-2022 The MathWorks, Inc.

desktop_solver = 'ode15s';

realtime_nonlinIter = '3';
realtime_stepSize = '0.001';
realtime_localSolver = 'NE_BACKWARD_EULER_ADVANCER';
realtime_globalSolver = 'ode3';

solverBlock_pth = find_system(mdl,'FollowLinks','on','LookUnderMasks','on', 'SubClassName', 'solver');

if strcmpi(deskreal,'desktop')
    set_param(mdl,'Solver',desktop_solver);    
    for svb_i=1:size(solverBlock_pth,1)
        set_param(char(solverBlock_pth(svb_i)), 'UseLocalSolver','off','DoFixedCost','off');
    end
else
    set_param(mdl,'Solver',realtime_globalSolver,'FixedStep',realtime_stepSize);
    for svb_i=1:size(solverBlock_pth,1)
        set_param(char(solverBlock_pth(svb_i)),...
            'UseLocalSolver','on',...
            'DoFixedCost','on',...
            'MaxNonlinIter',realtime_nonlinIter,...
            'LocalSolverChoice',realtime_localSolver,...
            'LocalSolverSampleTime',realtime_stepSize);
    end
end
