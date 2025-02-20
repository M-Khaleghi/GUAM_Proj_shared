function [AeroModel] = LC_TransV_17F_Quad_FM_alph0p05_fun(facs)
% Usage:    [AeroModel] = LC_TransV_17F_Quad_FM_alph0p05_fun(facs)
%
% Purpose:  This function contains the aerodynamic model for Lift+Cruise.
%           The models are given as polynomial expressions for each of the
%           aerodynamic forces and moments as a function of the state and
%           control variables.
%
%           This code was automatically generated using "MakeAeroFunction_LpC_17fac_check_hold.m"
%           from "LC_TransV_17F_Quad_FM_alph0p05.xlsx" on 14-Sep-2020 18:57:49.
%
% Inputs: 
%           facs:       vector of 17 factors for Lift+Cruise (states & controls)
%                       See the variable definitions in the code below (The units are 
%                       in kts, degrees, and RPM)
% Outputs: 
%           AeroModel:  matrix of aerodynamic forces [lbf] and moments [ft-lbf]
%                       with column order: [Fnormal Faxial Mpitch Fside Mroll Myaw Flift Fdrag]; 
%
% Calls:
%           None. 
%
% Authors:   Patrick C. Murphy and Benjamin M. Simmons
%
% History:  
%           February 03, 2020 - Created and debugged for Lift+Cruise implementation, BMS. 
%           June 22, 2020 - Added check for model validity, BMS. 
%

% Extract variables from the "facs" input
u = facs(1); % x-body axis velocity [kts]
v = facs(2); % y-body axis velocity [kts]
w = facs(3); % z-body axis velocity [kts]
LA = facs(4); % Left Aileron Deflection [deg]
RA = facs(5); % Right Aileron Deflection [deg]
LE = facs(6); % Left Elevator Deflection [deg]
RE = facs(7); % Right Elevator Deflection [deg]
RUD = facs(8); % Rudder Deflection [deg]
N1 = facs(9); % Engine 1 Speed [RPM]
N2 = facs(10); % Engine 2 Speed [RPM]
N3 = facs(11); % Engine 3 Speed [RPM]
N4 = facs(12); % Engine 4 Speed [RPM]
N5 = facs(13); % Engine 5 Speed [RPM]
N6 = facs(14); % Engine 6 Speed [RPM]
N7 = facs(15); % Engine 7 Speed [RPM]
N8 = facs(16); % Engine 8 Speed [RPM]
N9 = facs(17); % Engine 9 Speed [RPM]

% Check factor limits - if model limits are exceeded hold the boundary value.
u=check_fac_limits(u, -5, 25, 2.0);
v=check_fac_limits(v, -2, 2, 2.0);
w=check_fac_limits(w, -10, 10, 2.0);
LA=check_fac_limits(LA, -30, 30, 2.0);
RA=check_fac_limits(RA, -30, 30, 2.0);
LE=check_fac_limits(LE, -30, 30, 2.0);
RE=check_fac_limits(RE, -30, 30, 2.0);
RUD=check_fac_limits(RUD, -30, 30, 2.0);
N1=check_fac_limits(N1, 550, 1550, 2.0);
N2=check_fac_limits(N2, 550, 1550, 2.0);
N3=check_fac_limits(N3, 550, 1550, 2.0);
N4=check_fac_limits(N4, 550, 1550, 2.0);
N5=check_fac_limits(N5, 550, 1550, 2.0);
N6=check_fac_limits(N6, 550, 1550, 2.0);
N7=check_fac_limits(N7, 550, 1550, 2.0);
N8=check_fac_limits(N8, 550, 1550, 2.0);
N9=check_fac_limits(N9, 750, 1750, 2.0);

% Compute force and moment values
Fnormal = ...
+ 2.2061756000e+02 ...
+ 9.1181700000e+00 * u ...
+ 3.2608900000e+01 * v ...
+ 2.4699880000e+01 * w ...
+ 9.8320100000e-01 * LA ...
+ -8.4049200000e-01 * RA ...
+ -8.9267300000e-01 * LE ...
+ -2.4135200000e+00 * RE ...
+ -1.9193300000e+00 * RUD ...
+ 1.7559000000e-01 * N1 ...
+ 2.8272900000e-01 * N2 ...
+ 1.6611400000e-01 * N3 ...
+ 2.6163700000e-01 * N4 ...
+ 8.8969000000e-02 * N5 ...
+ -4.3974500000e-01 * N6 ...
+ -3.1230500000e-01 * N7 ...
+ -7.8698000000e-02 * N8 ...
+ -1.8324900000e-01 * N9 ...
+ 2.7762600000e+00 * u * w ...
+ 1.0637500000e-01 * u * LA ...
+ 9.5463000000e-02 * u * RA ...
+ 4.7089000000e-02 * u * RE ...
+ -7.7740000000e-03 * u * N1 ...
+ 1.1619000000e-02 * u * N2 ...
+ -1.6840000000e-02 * u * N3 ...
+ 1.2598000000e-02 * u * N4 ...
+ -1.8190000000e-02 * u * N5 ...
+ 1.1983000000e-02 * u * N6 ...
+ -8.2780000000e-03 * u * N7 ...
+ 1.3321000000e-02 * u * N8 ...
+ 2.3226000000e-02 * v * N2 ...
+ -3.1870000000e-02 * v * N5 ...
+ -2.7425000000e-02 * v * N8 ...
+ -8.9730000000e-03 * w * N1 ...
+ 9.5670000000e-03 * w * N4 ...
+ 4.9170000000e-03 * w * N6 ...
+ -8.0640000000e-03 * w * N7 ...
+ 7.8480000000e-03 * w * N8 ...
+ -2.5929000000e-02 * LA * RUD ...
+ 1.5440000000e-03 * RA * N3 ...
+ -1.5000000000e-03 * LE * N8 ...
+ 3.7680000000e-03 * LE * N9 ...
+ -1.6770000000e-03 * RE * N2 ...
+ 1.4350000000e-03 * RE * N7 ...
+ 3.0300000000e-03 * RE * N9 ...
+ 1.7500000000e-03 * RUD * N5 ...
+ -9.3000000000e-05 * N1 * N2 ...
+ -9.7000000000e-05 * N1 * N8 ...
+ -1.2200000000e-04 * N2 * N3 ...
+ 1.6000000000e-04 * N4 * N6 ...
+ 1.0100000000e-04 * N6 * N7 ...
+ 1.6300000000e-04 * N6 * N8 ...
+ 7.1334100000e-01 * u^2 ...
+ -6.3364000000e-02 * RE^2 ...
+ 8.3300000000e-04 * N1^2 ...
+ 8.3900000000e-04 * N2^2 ...
+ 8.0600000000e-04 * N3^2 ...
+ 6.9700000000e-04 * N4^2 ...
+ 7.7700000000e-04 * N5^2 ...
+ 9.0200000000e-04 * N6^2 ...
+ 9.1900000000e-04 * N7^2 ...
+ 8.9500000000e-04 * N8^2;

Faxial = ...
+ 5.7319394000e+02 ...
+ 8.4437800000e+00 * u ...
+ -1.4012200000e+01 * v ...
+ -8.8180600000e+00 * w ...
+ -8.3931000000e-02 * LA ...
+ -1.0613200000e+00 * RA ...
+ 1.0783200000e+00 * LE ...
+ -1.0142800000e+00 * RE ...
+ -8.9864600000e-01 * RUD ...
+ 3.1363000000e-02 * N1 ...
+ -2.2857100000e-01 * N2 ...
+ -4.2835000000e-02 * N3 ...
+ 2.0915000000e-02 * N4 ...
+ -4.0288300000e-01 * N5 ...
+ 1.8273000000e-02 * N6 ...
+ -4.6610000000e-03 * N7 ...
+ 3.7777000000e-02 * N8 ...
+ -8.7408900000e-01 * N9 ...
+ 1.3096600000e+00 * u * v ...
+ -4.3754500000e-01 * u * w ...
+ 2.6202000000e-02 * u * LA ...
+ 3.0210000000e-02 * u * RA ...
+ 2.4617000000e-02 * u * RE ...
+ 4.3690000000e-03 * u * N1 ...
+ 3.2610000000e-03 * u * N2 ...
+ 4.3290000000e-03 * u * N3 ...
+ 6.4320000000e-03 * u * N4 ...
+ 3.1830000000e-03 * u * N5 ...
+ -1.7590000000e-03 * u * N6 ...
+ 3.1890000000e-03 * u * N7 ...
+ -6.8700000000e-03 * u * N9 ...
+ -1.6360200000e+00 * v * w ...
+ 1.2905000000e-02 * v * N6 ...
+ -1.7854000000e-02 * v * N9 ...
+ 3.3830000000e-03 * w * N5 ...
+ 4.8080000000e-03 * w * N9 ...
+ -1.2184000000e-02 * RA * LE ...
+ 7.3000000000e-04 * RA * N8 ...
+ 1.4490000000e-02 * LE * RUD ...
+ -8.8000000000e-04 * LE * N2 ...
+ -1.3210000000e-03 * LE * N9 ...
+ 1.6510000000e-03 * RE * N9 ...
+ 2.1340000000e-03 * RUD * N9 ...
+ -4.6000000000e-05 * N1 * N4 ...
+ -7.3000000000e-05 * N4 * N6 ...
+ 1.0000000000e-04 * N4 * N9 ...
+ 6.9000000000e-05 * N5 * N6 ...
+ -4.9136100000e-01 * u^2 ...
+ -3.6490800000e-01 * w^2 ...
+ 3.8065000000e-02 * RE^2 ...
+ 1.1300000000e-04 * N2^2 ...
+ 1.2900000000e-04 * N5^2 ...
+ -9.9300000000e-04 * N9^2;

Mpitch = ...
+ -1.7880818500e+03 ...
+ 5.3222100000e+01 * u ...
+ -6.8373260000e+01 * v ...
+ -2.2664364000e+02 * w ...
+ 3.9417000000e-01 * LA ...
+ 9.8433400000e+00 * RA ...
+ 2.2487300000e+01 * LE ...
+ 1.0051030000e+01 * RE ...
+ 3.9472500000e+00 * RUD ...
+ 5.6013600000e-01 * N1 ...
+ 6.5054500000e-01 * N2 ...
+ 1.4124000000e+00 * N3 ...
+ 2.0100000000e-01 * N4 ...
+ 1.9801800000e+00 * N5 ...
+ -1.5607000000e-01 * N6 ...
+ 1.0677100000e+00 * N7 ...
+ -1.1024900000e+00 * N8 ...
+ -1.2901500000e+00 * N9 ...
+ 3.5678700000e+00 * u * w ...
+ -3.6140500000e-01 * u * LE ...
+ -3.7619100000e-01 * u * RE ...
+ 3.3321000000e-02 * u * N3 ...
+ 3.7978000000e-02 * u * N4 ...
+ 3.4689000000e-02 * u * N5 ...
+ 3.3276000000e-02 * u * N6 ...
+ -1.2548000000e-02 * u * N9 ...
+ -8.7557000000e-02 * v * N2 ...
+ -8.2064000000e-02 * v * N5 ...
+ 9.0953000000e-02 * v * N7 ...
+ 1.2541000000e-01 * v * N8 ...
+ 3.9178000000e-02 * w * N3 ...
+ 2.2685000000e-02 * w * N4 ...
+ 4.0080000000e-02 * w * N5 ...
+ 5.2640000000e-02 * w * N6 ...
+ 2.1903000000e-02 * w * N9 ...
+ 1.0551900000e-01 * LA * LE ...
+ -5.9230000000e-03 * LA * N7 ...
+ 6.0370000000e-03 * LA * N8 ...
+ -7.6600000000e-03 * RA * N4 ...
+ -1.9252100000e-01 * LE * RE ...
+ -5.5700000000e-03 * LE * N4 ...
+ -7.1550000000e-03 * LE * N6 ...
+ -1.9736000000e-02 * LE * N9 ...
+ -1.4770000000e-02 * RE * N9 ...
+ 7.5100000000e-04 * N1 * N3 ...
+ 5.3000000000e-04 * N2 * N9 ...
+ 4.4200000000e-04 * N3 * N6 ...
+ 6.6400000000e-04 * N5 * N7 ...
+ 3.2800000000e-04 * N5 * N8 ...
+ -3.9728800000e+00 * u^2 ...
+ -5.7527290000e+01 * v^2 ...
+ 4.6970300000e+00 * w^2 ...
+ 2.0621400000e-01 * LE^2 ...
+ 2.4960700000e-01 * RUD^2 ...
+ 2.5550000000e-03 * N1^2 ...
+ -3.8790000000e-03 * N2^2 ...
+ 1.9040000000e-03 * N3^2 ...
+ -3.0140000000e-03 * N4^2 ...
+ 1.7160000000e-03 * N5^2 ...
+ -2.9690000000e-03 * N6^2 ...
+ 2.4410000000e-03 * N7^2 ...
+ -2.9010000000e-03 * N8^2 ...
+ -1.1410000000e-03 * N9^2;

Fside = ...
+ 9.6036190000e+01 ...
+ 1.6835600000e+00 * u ...
+ 2.2242010000e+01 * v ...
+ -9.2457100000e-01 * w ...
+ -1.1548300000e-01 * LA ...
+ 1.3822000000e+00 * RA ...
+ 3.0241500000e-01 * LE ...
+ 2.2925500000e-01 * RE ...
+ 6.1082100000e-01 * RUD ...
+ 2.0045000000e-02 * N1 ...
+ -3.2085000000e-02 * N2 ...
+ -2.0788000000e-02 * N3 ...
+ -3.3977700000e-01 * N4 ...
+ 3.3404000000e-02 * N5 ...
+ 8.6956000000e-02 * N6 ...
+ -2.1589000000e-02 * N7 ...
+ 2.3119000000e-02 * N8 ...
+ 1.1683000000e-02 * N9 ...
+ -7.7704000000e-02 * u * RUD ...
+ -1.8050000000e-03 * u * N3 ...
+ -2.6140000000e-03 * u * N4 ...
+ 2.8290000000e-03 * u * N6 ...
+ -9.1200000000e-04 * u * N9 ...
+ -7.6580000000e-03 * v * N1 ...
+ -1.0102000000e-02 * v * N4 ...
+ -6.8200000000e-03 * v * N5 ...
+ -7.9680000000e-03 * v * N6 ...
+ -6.7430000000e-03 * v * N8 ...
+ -1.5027000000e-02 * v * N9 ...
+ 3.5022000000e-02 * w * RA ...
+ -3.7880000000e-03 * w * N2 ...
+ -2.7110000000e-03 * w * N4 ...
+ 2.8110000000e-03 * w * N6 ...
+ 1.5810000000e-03 * w * N7 ...
+ 2.9660000000e-03 * w * N8 ...
+ -4.9700000000e-04 * LA * N1 ...
+ 5.0000000000e-04 * LA * N7 ...
+ -6.6900000000e-04 * RA * N7 ...
+ -5.9200000000e-04 * RA * N8 ...
+ -4.7200000000e-04 * LE * N4 ...
+ -5.5300000000e-04 * RE * N1 ...
+ 4.9600000000e-04 * RE * N5 ...
+ 8.1100000000e-04 * RUD * N4 ...
+ -4.7800000000e-04 * RUD * N5 ...
+ -2.0530000000e-03 * RUD * N9 ...
+ -4.6000000000e-05 * N1 * N3 ...
+ 5.8000000000e-05 * N3 * N4 ...
+ -4.3000000000e-05 * N5 * N6 ...
+ 5.5000000000e-05 * N5 * N7 ...
+ 3.8000000000e-05 * N6 * N9 ...
+ -4.4494300000e+00 * v^2 ...
+ -1.5300000000e-04 * N3^2 ...
+ 1.4200000000e-04 * N5^2 ...
+ 9.7000000000e-05 * N6^2;

Mroll = ...
+ 4.9968738900e+03 ...
+ 2.6506900000e+00 * u ...
+ -7.8390182000e+02 * v ...
+ 1.8220940000e+01 * w ...
+ 2.7895770000e+01 * LA ...
+ 3.2280290000e+01 * RA ...
+ 1.6524170000e+01 * LE ...
+ -4.4957910000e+01 * RUD ...
+ 1.1702600000e+00 * N1 ...
+ -4.0512800000e+00 * N2 ...
+ -3.0753900000e+00 * N3 ...
+ -8.5228600000e-01 * N4 ...
+ 3.2768000000e+00 * N5 ...
+ 4.0598000000e+00 * N6 ...
+ -4.2926200000e+00 * N7 ...
+ -7.6564500000e-01 * N8 ...
+ -3.8155900000e+00 * N9 ...
+ 4.2645740000e+01 * u * v ...
+ 1.3396700000e+00 * u * LA ...
+ -1.2181000000e+00 * u * RA ...
+ -2.4768500000e-01 * u * N1 ...
+ 1.7977400000e-01 * u * N2 ...
+ -2.0526900000e-01 * u * N3 ...
+ 4.0686000000e-02 * u * N4 ...
+ 1.6469300000e-01 * u * N5 ...
+ -3.2641000000e-02 * u * N6 ...
+ 2.3021500000e-01 * u * N7 ...
+ -1.5403900000e-01 * u * N8 ...
+ -7.0752700000e+01 * v * w ...
+ 2.6303800000e-01 * v * N2 ...
+ -2.2765300000e-01 * v * N3 ...
+ -2.2566000000e-01 * v * N4 ...
+ 2.0190900000e-01 * v * N8 ...
+ -2.2687400000e-01 * v * N9 ...
+ -2.3805400000e-01 * w * N1 ...
+ 4.9016000000e-02 * w * N5 ...
+ 1.7934700000e-01 * w * N7 ...
+ -3.6251600000e-01 * LA * RUD ...
+ -1.5786000000e-02 * LA * N1 ...
+ -2.4139000000e-02 * RA * N2 ...
+ -1.5121000000e-02 * RA * N5 ...
+ -1.5877000000e-02 * RA * N7 ...
+ 1.5178000000e-02 * RA * N9 ...
+ -1.4770000000e-02 * LE * N3 ...
+ 1.3416000000e-02 * RUD * N3 ...
+ 1.6395000000e-02 * RUD * N4 ...
+ 1.1360000000e-03 * N2 * N9 ...
+ -8.6000000000e-04 * N3 * N6 ...
+ -1.2880000000e-03 * N3 * N8 ...
+ 1.0750000000e-03 * N5 * N7 ...
+ 1.3720000000e-03 * N6 * N7 ...
+ -1.0050000000e-03 * N6 * N9 ...
+ 2.7050000000e-03 * N7 * N8 ...
+ -9.1800000000e-04 * N8 * N9 ...
+ 1.5130000000e-02 * N1^2 ...
+ 1.7634000000e-02 * N2^2 ...
+ 9.0100000000e-03 * N3^2 ...
+ 7.2370000000e-03 * N4^2 ...
+ -8.2100000000e-03 * N5^2 ...
+ -8.4470000000e-03 * N6^2 ...
+ -1.6270000000e-02 * N7^2 ...
+ -1.6436000000e-02 * N8^2;

Myaw = ...
+ -1.8910513900e+03 ...
+ -1.0273570000e+01 * u ...
+ -2.8081023000e+02 * v ...
+ -7.8387270000e+01 * w ...
+ 5.9988000000e+00 * LA ...
+ -6.8578800000e+00 * RA ...
+ -3.9095300000e+00 * LE ...
+ -8.8352400000e+00 * RE ...
+ 6.7115100000e+00 * RUD ...
+ -4.2852900000e-01 * N1 ...
+ 4.9089400000e+00 * N2 ...
+ 2.2351000000e+00 * N3 ...
+ 1.9658800000e+00 * N4 ...
+ -1.7019300000e+00 * N5 ...
+ 7.2020700000e-01 * N6 ...
+ -2.5262200000e+00 * N7 ...
+ -4.7921000000e+00 * N8 ...
+ 3.0369400000e+00 * N9 ...
+ 7.4423900000e-01 * u * w ...
+ -3.7901800000e-01 * u * LA ...
+ 5.9923300000e-01 * u * RA ...
+ 1.1217500000e+00 * u * RUD ...
+ -7.3030000000e-02 * u * N1 ...
+ -5.9422000000e-02 * u * N2 ...
+ -1.7244000000e-02 * u * N3 ...
+ 3.0480000000e-02 * u * N4 ...
+ 2.7067000000e-02 * u * N5 ...
+ -2.2021000000e-02 * u * N6 ...
+ 6.5477000000e-02 * u * N7 ...
+ 5.9821000000e-02 * u * N8 ...
+ 1.7149000000e-02 * u * N9 ...
+ -1.2264900000e-01 * v * N2 ...
+ 1.1511300000e-01 * v * N4 ...
+ 1.1482800000e-01 * v * N6 ...
+ 2.6560600000e-01 * v * N9 ...
+ -3.7455000000e-02 * w * N1 ...
+ 4.2940000000e-02 * w * N2 ...
+ 1.9951000000e-02 * w * N3 ...
+ 3.1568000000e-02 * w * N7 ...
+ 1.3922900000e-01 * LA * RE ...
+ 7.8110000000e-03 * LA * N1 ...
+ -6.8040000000e-03 * LA * N7 ...
+ -1.3192600000e-01 * RA * RE ...
+ 7.1420000000e-03 * LE * N9 ...
+ 7.8240000000e-03 * RE * N1 ...
+ -7.4750000000e-03 * RUD * N4 ...
+ -1.1730000000e-02 * RUD * N6 ...
+ 3.1991000000e-02 * RUD * N9 ...
+ -4.0800000000e-04 * N1 * N2 ...
+ -4.3700000000e-04 * N2 * N8 ...
+ -1.2090000000e-03 * N3 * N4 ...
+ 4.0300000000e-04 * N5 * N6 ...
+ -6.2200000000e-04 * N6 * N9 ...
+ 6.5400000000e-04 * N7 * N8 ...
+ 1.2710000000e-03 * N1^2 ...
+ -2.4090000000e-03 * N2^2 ...
+ -2.0290000000e-03 * N3^2 ...
+ 1.3560000000e-03 * N4^2 ...
+ 2.2150000000e-03 * N5^2 ...
+ -1.8930000000e-03 * N6^2 ...
+ 2.7780000000e-03 * N8^2 ...
+ -1.1710000000e-03 * N9^2;

Flift = ...
+ 3.3125681000e+03 ...
+ 1.8567464000e+02 * u ...
+ -4.1201410000e+01 * w ...
+ 4.3226500000e-01 * RE ...
+ -2.3545100000e+00 * RUD ...
+ -3.8443000000e-02 * N1 ...
+ -3.4561700000e+00 * N2 ...
+ 3.5738700000e-01 * N3 ...
+ 1.8503800000e-01 * N4 ...
+ 1.4677400000e-01 * N5 ...
+ -3.5870200000e-01 * N6 ...
+ -2.3248100000e-01 * N7 ...
+ 2.7060300000e-01 * N8 ...
+ -8.3051000000e-02 * N9 ...
+ -2.4619200000e+00 * u * w ...
+ 6.6727000000e-02 * u * N1 ...
+ 9.9330000000e-02 * u * N2 ...
+ 6.3705000000e-02 * u * N3 ...
+ 8.2053000000e-02 * u * N4 ...
+ 7.3833000000e-02 * u * N5 ...
+ 1.0401300000e-01 * u * N6 ...
+ 7.2228000000e-02 * u * N7 ...
+ 1.0261100000e-01 * u * N8 ...
+ 2.2470600000e-01 * w * N9 ...
+ -5.2000000000e-04 * N2 * N5 ...
+ -5.0500000000e-04 * N3 * N8 ...
+ -2.2242820000e+01 * u^2 ...
+ -3.7596600000e-01 * RE^2 ...
+ -3.9860900000e-01 * RUD^2 ...
+ 1.7260000000e-03 * N2^2;

Fdrag = ...
+ -9.0125221000e+02 ...
+ -4.7695300000e+01 * u ...
+ -2.3274442000e+02 * w ...
+ -4.1333000000e-02 * N1 ...
+ -3.2195000000e-02 * N2 ...
+ -2.8945000000e-02 * N3 ...
+ 1.8732500000e-01 * N4 ...
+ 1.2839500000e-01 * N5 ...
+ -1.2152000000e-02 * N6 ...
+ 1.6499700000e-01 * N7 ...
+ 1.1857300000e-01 * N8 ...
+ 1.8421000000e-01 * N9 ...
+ -1.7679380000e+01 * u * w ...
+ -1.5283400000e-01 * u * N9 ...
+ 1.1938900000e-01 * w * N1 ...
+ 1.2912700000e-01 * w * N2 ...
+ 1.0443000000e-01 * w * N3 ...
+ 1.3342100000e-01 * w * N4 ...
+ 1.1480100000e-01 * w * N5 ...
+ 1.3747700000e-01 * w * N6 ...
+ 1.1823300000e-01 * w * N7 ...
+ 1.3336600000e-01 * w * N8 ...
+ 6.6865500000e+00 * u^2 ...
+ 3.6817600000e+00 * w^2;

% Aerodynamic forces [lbf] and moments [ft-lbf]
AeroModel = [Fnormal Faxial Mpitch Fside Mroll Myaw Flift Fdrag];

return
