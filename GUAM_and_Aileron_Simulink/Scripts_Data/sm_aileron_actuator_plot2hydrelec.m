% Code to plot simulation results from sm_aileron_actuator
%% Plot Description:
%
% The plots below compare the performance of the hydraulic and electric
% designs with the desired performance.
%
% Copyright 2018-2022 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
if ~exist('h2_sm_aileron_actuator', 'var') || ...
        ~isgraphics(h2_sm_aileron_actuator, 'figure')
    h2_sm_aileron_actuator = figure('Name', 'sm_aileron_actuator');
end
figure(h2_sm_aileron_actuator)
clf(h2_sm_aileron_actuator)

temp_colororder = get(gca,'defaultAxesColorOrder');

config_set = {'Motion','Hydraulic','E SL Avg'};

for i=1:length(config_set)
    sm_aileron_actuator_configModel(bdroot,config_set{i})
    sim(bdroot)
    % Get simulation results
    simlog_t    = simlog_sm_aileron_actuator.Mechanical.Cyl_MC_BrkLAil.Rz.q.series.time;
    simlog_qAil = simlog_sm_aileron_actuator.Mechanical.Cyl_MC_BrkLAil.Rz.q.series.values('deg');
    simlog_actFrc = logsout_sm_aileron_actuator.get('force_act');
    simlog_qCmd = logsout_sm_aileron_actuator.get('qCmd');
    
    % Plot results
    simlog_handles(1) = subplot(2, 1, 1);
    hold on
    if(i==1)
        plot(simlog_qCmd.Values.Time, simlog_qCmd.Values.Data, 'k-.', 'LineWidth', 1)
    end
    plot(simlog_t, simlog_qAil, 'LineWidth', 1,'Color',temp_colororder(i,:));
    
    simlog_handles(2) = subplot(2, 1, 2);
    hold on
    plot(simlog_actFrc.Values.Time, simlog_actFrc.Values.Data, 'LineWidth', 1)
end
grid(simlog_handles(1),'on');
box(simlog_handles(1),'on');
grid(simlog_handles(2),'on');
box(simlog_handles(2),'on');

title(simlog_handles(1),'Aileron Angle')
ylabel(simlog_handles(1),'Angle (deg)')
config_set = {'Motion','Hydraulic','Electric'};

legend(simlog_handles(1),{'Command',config_set{:}},'Location','North');

title(simlog_handles(2),'Actuator Force')
ylabel(simlog_handles(2),'Force (N)')
xlabel(simlog_handles(2),'Time (s)')

linkaxes(simlog_handles, 'x')

hold(simlog_handles(1),'off')
hold(simlog_handles(2),'off')

% Remove temporary variables
clear simlog_t simlog_handles temp_colororder
clear simlog_actFrc simlog_qCmd simlog_qAil config_set 
