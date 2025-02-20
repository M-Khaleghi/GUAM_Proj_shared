function sm_aileron_actuator_configEDriver(modelname,config_name)
% Copyright 2012-2022 The MathWorks(TM), Inc.

f    = Simulink.FindOptions('FollowLinks',1,'LookUnderMasks','none');
actuate_path = getfullname(Simulink.findBlocks(bdroot,'Name','Actuator',f));
econtrl_path = find_system(modelname,'FollowLinks','on','regexp','on','MatchFilter',@Simulink.match.allVariants,'Name','Leadscrew.*');

switch config_name
    case 'Averaged'
        set_param(actuate_path,'OverrideUsingVariant','Electric');
        for i=1:length(econtrl_path)
            set_param(econtrl_path{i},'popup_driver','Averaged');
        end
    case 'PWM'
        set_param(actuate_path,'OverrideUsingVariant','Electric');
        for i=1:length(econtrl_path)
            set_param(econtrl_path{i},'popup_driver','PWM');
        end
    case 'Circuit'
        set_param(actuate_path,'OverrideUsingVariant','Electric');
        for i=1:length(econtrl_path)
            set_param(econtrl_path{i},'popup_driver','Circuit');
        end
    otherwise
        warning('Unexpected Configuration setting.')
end

set_param(actuate_path,'Name',get_param(actuate_path,'Name'));
