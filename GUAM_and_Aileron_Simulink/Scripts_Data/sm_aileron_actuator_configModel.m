function sm_aileron_actuator_configModel(modelname,config_name)
% Copyright 2012-2022 The MathWorks(TM), Inc.

f    = Simulink.FindOptions('FollowLinks',1,'LookUnderMasks','none');
actuate_path = getfullname(Simulink.findBlocks(bdroot,'Name','Actuator',f));
econtrl_path = find_system(modelname,'FollowLinks','on','regexp','on','MatchFilter',@Simulink.match.allVariants,'Name','Leadscrew.*');

switch config_name
    case 'Motion'
        set_param(actuate_path,'LabelModeActiveChoice','Motion');
    case 'Ideal'
        set_param(actuate_path,'LabelModeActiveChoice','Ideal');
    case 'Hydraulic'
        set_param(actuate_path,'LabelModeActiveChoice','Hydraulic');
    case 'E SL Avg'
        set_param(actuate_path,'LabelModeActiveChoice','Electric');
        for i=1:length(econtrl_path)
            set_param(econtrl_path{i},'popup_ctrl','Simulink','popup_driver','Averaged')
        end
    case 'E SL PWM'
        set_param(actuate_path,'LabelModeActiveChoice','Electric');
        for i=1:length(econtrl_path)
            set_param(econtrl_path{i},'popup_ctrl','Simulink','popup_driver','PWM')
        end
    case 'E SL Cir'
        set_param(actuate_path,'LabelModeActiveChoice','Electric');
        for i=1:length(econtrl_path)
            set_param(econtrl_path{i},'popup_ctrl','Simulink','popup_driver','Circuit')
        end
    case 'E Cir Avg'
        set_param(actuate_path,'LabelModeActiveChoice','Electric');
        for i=1:length(econtrl_path)
            set_param(econtrl_path{i},'popup_ctrl','Circuit','popup_driver','Averaged')
        end
    case 'E Cir PWM'
        set_param(actuate_path,'LabelModeActiveChoice','Electric');
        for i=1:length(econtrl_path)
            set_param(econtrl_path{i},'popup_ctrl','Circuit','popup_driver','PWM')
        end
	case 'E Cir Cir'
        set_param(actuate_path,'LabelModeActiveChoice','Electric');
        for i=1:length(econtrl_path)
            set_param(econtrl_path{i},'popup_ctrl','Circuit','popup_driver','Circuit')
        end
    otherwise
        warning('Unexpected Configuration setting.')
end

set_param(actuate_path,'Name',get_param(actuate_path,'Name'));
