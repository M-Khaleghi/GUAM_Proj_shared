%% sim parameters

model = 'GUAM';

% use timeseries input
userStruct.variants.refInputType=3; % 1=FOUR_RAMP, 2= ONE_RAMP, 3=Timeseries, 4=Default(doublets)

%% setup trajectory and pass to target
time        = [0 5:0.5:20]'; % Column vector of time points
N_time = length(time);

vel     = zeros(N_time, 3);
vel_i   = zeros(N_time, 3);
pos     = zeros(N_time, 3);
chi     = zeros(N_time, 3);
chid    = zeros(N_time, 3);

% define trajectory as climbing right hand turn for 90 deg
% prescibe inertial position (NED)
pos     = [0 0 -500]; % Inertial Positions (x,y,-z) row vector for each time
cir_xy = [1000;0]'+[sind(0:3:90)*2000; cosd(0:3:90)*-2000+2000]';
pos = [pos;[cir_xy [0:30]'*-100/30-500]];
%pos = [pos;[cir_xy [0:22]'*-300/30-500]; [5997.6 3880.4 -721]; [5997.6 4080.4 -729]; [5997.6 4280.4 -737]; [5997.6 4480.4 -744]; [5997.6 4680.4 -751]; [5997.6 4880.4 -758]; [5997.6 5080.4 -765]; [5997.6 5280.4 -772]]; %%The -1000 at the end is the initial value, the -1000/30 is the amount of increment per step
%pos = [pos;[cir_xy [0:22]'*-0/23-500]; [8996.3 5800.6 -500]; [8996.3 5900.4 -500]; [8996.3 6000.4 -500]; [8996.3 6100.4 -500]; [8996.3 6200.4 -500]; [8996.3 6300.4 -500]; [8996.3 6400.4 -500]; [8996.3 6500.4 -500]];
%pos = [pos;[cir_xy [0:22]'*-0/23-500]; [11996 7922 -500]; [11996 8322 -500]; [11996.3 8722 -500]; [11996.3 9122 -500]; [11996.3 9522 -500]; [11996.3 9922 -500]; [11996.3 10322 -500]; [11996.3 10722 -500]];
%pos = [pos;[cir_xy (0:22)'*-0/23-500];[5997.6 3880.4 -500]; [5997.6 4080.4 -500]; [5997.6 4280.4 -500]; [5997.6 4480.4 -500]; [5997.6 4680.4 -500]; [5997.6 4880.4 -500]; [5997.6 5080.4 -500]; [5997.6 5280.4 -500]];
%pos = [pos;[cir_xy (0:22)'*-90/23-500]; [5997.6 3880.4 -586.1]; [5997.6 4080.4 -590]; [5997.6 4280.4 -594]; [5997.6 4480.4 -598]; [5997.6 4680.4 -602]; [5997.6 4880.4 -606]; [5997.6 5080.4 -610]; [5997.6 5280.4 -614]];

% Compute velocity
vel_i(:,1)  = gradient(pos(:,1))./gradient(time); 
vel_i(:,2)  = gradient(pos(:,2))./gradient(time); 
vel_i(:,3)  = gradient(pos(:,3))./gradient(time); 

% Compute heading
chi     = atan2(vel_i(:,2),vel_i(:,1));
chid    = gradient(chi)./gradient(time);

% add stars library blocks for quaternion functions
addpath(genpath('lib'));

% compute velocity in heading frame
q = QrotZ(chi);
vel = Qtrans(q,vel_i);

% setup trajectory to match bus
RefInput.Vel_bIc_des    = timeseries(vel,time); % Heading frame velocity
RefInput.pos_des        = timeseries(pos,time); % Inertial Position
RefInput.chi_des        = timeseries(chi,time); % Heading Angle
RefInput.chi_dot_des    = timeseries(chid,time); % Heading Angle Rate
RefInput.vel_des        = timeseries(vel_i,time); % Inertial Position

target.RefInput = RefInput;

%% Prepare to run simulation
% set initial conditions and add trajectory to SimInput
simSetup;

open(model);
%open(model2);
