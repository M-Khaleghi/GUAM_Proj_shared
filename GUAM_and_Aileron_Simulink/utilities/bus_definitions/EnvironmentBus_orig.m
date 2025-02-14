function cellInfo = EnvironmentBus(varargin) 
% ENVIRONMENTBUS returns a cell array containing bus object information 
% 
% Optional Input: 'false' will suppress a call to Simulink.Bus.cellToObject 
%                 when the MATLAB file is executed. 
% The order of bus element attributes is as follows:
%   ElementName, Dimensions, DataType, SampleTime, Complexity, SamplingMode, DimensionsMode, Min, Max, DocUnits, Description 

suppressObject = false; 
if nargin == 1 && islogical(varargin{1}) && varargin{1} == false 
    suppressObject = true; 
elseif nargin > 1 
    error('Invalid input argument(s) encountered'); 
end 

cellInfo = { ... 
  { ... 
    'EnvironmentBus', ... 
    'BusEnvironment.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'Wind', 1, 'Bus: WindBus', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Turbulence', 1, 'Bus: TurbulenceBus', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Atmosphere', 1, 'Bus: AtmosphereBus', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
  { ... 
    'WindBus', ... 
    'BusWind.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'Vel_wHh', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'VelDtH_wHh', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
  { ... 
    'TurbulenceBus', ... 
    'BusTurbulence.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'Vel_tBb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'VelDtB_tBb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Omeg_TBb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'OmegDtB_TBb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
  { ... 
    'AtmosphereBus', ... 
    'BusAtmosphere.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'Density', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Pressure', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Temperature', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'SpeedOfSound', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
}'; 

if ~suppressObject 
    % Create bus objects in the MATLAB base workspace 
    Simulink.Bus.cellToObject(cellInfo) 
end 
