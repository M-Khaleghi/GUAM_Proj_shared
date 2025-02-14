function cellInfo = FailureBusEng(varargin) 
% FO_FailureBus returns a cell array containing bus object information 
% 
% Optional Input: 'false' will suppress a call to Simulink.Bus.cellToObject 
%                 when the MATLAB file is executed. 
% The order of bus element attributes is as follows:
%   ElementName, Dimensions, DataType, Complexity, SamplingMode, DimensionsMode, Min, Max, DocUnits, Description 

suppressObject = false;
numEng=1;
numSurf=1;
if nargin >= 1
    if islogical(varargin{1}) && varargin{1} == false 
      suppressObject = true;
    end
    if nargin == 3
        numEng=varargin{2};
        numSurf=varargin{3};
    end
end 

cellInfo = { ... 
  { ... 
    'BUS_FAILURE_ENG', ... 
    '', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'F_Fail_Initiate', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Hold_Last', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Pre_Scale', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Post_Scale', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Pos_Bias', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Pos_Scale', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Up_Plim', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Lwr_Plim', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Rate_Bias', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Rate_Scale', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Up_Rlim', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Lwr_Rlim', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Accel_Bias', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Accel_Scale', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Up_Alim', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Lwr_Alim', numEng, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'F_Gen_Sig_Sel', 15, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
} ...
  } ...
}'; 

if ~suppressObject 
    % Create bus objects in the MATLAB base workspace 
    Simulink.Bus.cellToObject(cellInfo) 
end 
