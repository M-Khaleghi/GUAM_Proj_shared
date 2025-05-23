function sm_aileron_actuator_open_system(filename)
% Copyright 2016-2022 The MathWorks, Inc.

if (exist(filename))
    open_system(filename)
else
    msgbox(['File ' filename ' is not on your path.  It may have been excluded to reduce the size of the folder containing all the supporting files for this example.'],'File not provided');
end
