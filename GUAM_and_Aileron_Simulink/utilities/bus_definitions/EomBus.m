function cellInfo = EomBus(varargin) 
% EOMBUS returns a cell array containing bus object information 
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
    'EomBus', ... 
    'BusEom.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'InertialData', 1, 'Bus: InertialBus', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'WorldRelativeData', 1, 'Bus: WorldRelBus', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'AirRelativeData', 1, 'Bus: AirRelBus', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
  { ... 
    'InertialBus', ... 
    'BusInertial.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'Pos_bii', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Vel_bIi', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Accel_bIi', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Asensed_bIb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Q_i2b', 4, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Omeg_BIb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'OmegDtI_BIb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
  { ... 
    'WorldRelBus', ... 
    'BusWorldRel.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'Pos_bee', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Pos_beh_topo', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'LatLonAlt', 1, 'Bus: LatLonAltBus', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'AltMSL', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'AltAGL', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'AltPresMSL', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Euler', 1, 'Bus: EulerBus', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Q_h2b', 4, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Q_i2h', 4, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'gamma', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'chi', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Omeg_BEb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Omeg_BHb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Vel_bEb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Vel_bEh', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Omeg_HEh', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'VelDtE_bEb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'VelDtH_bEh', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'gammadot', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'chidot', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
  { ... 
    'LatLonAltBus', ... 
    'BusLatLonAlt.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'LatGeod', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Lon', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'AltGeod', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
  { ... 
    'EulerBus', ... 
    'BusEuler.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'phi', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'theta', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'psi', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
  { ... 
    'AirRelBus', ... 
    'BusAirRel.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'Vel_bWb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'VelDtB_bWb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Omeg_BWb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Vtot', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Veas', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Vcas', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Mach', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'qbar', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'qc', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'alpha', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'beta', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'mu', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'gamma', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'chi', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Vtotdot', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'alphadot', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'betadot', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'mudot', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'gammadot', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'chidot', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'alphaTotal', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'phiAero', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Q_h2v', 4, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Omeg_VHb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
}'; 

if ~suppressObject 
    % Create bus objects in the MATLAB base workspace 
    Simulink.Bus.cellToObject(cellInfo) 
end 
