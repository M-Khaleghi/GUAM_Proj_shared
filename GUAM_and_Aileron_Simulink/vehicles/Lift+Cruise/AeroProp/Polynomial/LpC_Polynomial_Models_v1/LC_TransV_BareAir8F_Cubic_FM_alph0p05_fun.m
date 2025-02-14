function [AeroModel] = LC_TransV_BareAir8F_Cubic_FM_alph0p05_fun(facs)
% Usage:    [AeroModel] = LC_TransV_BareAir8F_Cubic_FM_alph0p05_fun(facs)
%
% Purpose:  This function contains the aerodynamic model for Lift+Cruise.
%           The models are given as polynomial expressions for each of the
%           aerodynamic forces and moments as a function of the state and
%           control variables.
%
%           This code was automatically generated using "MakeAeroFunction_LpC_17fac_check_hold.m"
%           from "LC_TransV_BareAir8F_Cubic_FM_alph0p05.xlsx" on 14-Sep-2020 18:57:50.
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
N1=check_fac_limits(N1, 0, 0, 2.0);
N2=check_fac_limits(N2, 0, 0, 2.0);
N3=check_fac_limits(N3, 0, 0, 2.0);
N4=check_fac_limits(N4, 0, 0, 2.0);
N5=check_fac_limits(N5, 0, 0, 2.0);
N6=check_fac_limits(N6, 0, 0, 2.0);
N7=check_fac_limits(N7, 0, 0, 2.0);
N8=check_fac_limits(N8, 0, 0, 2.0);
N9=check_fac_limits(N9, 0, 0, 2.0);

% Compute force and moment values
Fnormal = ...
+ -3.2620400000e+00 ...
+ 3.7685500000e-01 * u ...
+ 2.1598330000e+01 * w ...
+ 1.1530900000e+00 * LA ...
+ 3.9385500000e-01 * RA ...
+ 3.6213900000e-01 * LE ...
+ -2.8561000000e-02 * RE ...
+ 2.2900100000e-01 * u * w ...
+ -2.6210000000e-03 * u * LA ...
+ -1.2541000000e-02 * u * RA ...
+ 2.2963000000e-02 * u * LE ...
+ -1.0491000000e-02 * u * RE ...
+ -3.3630000000e-03 * w * LA ...
+ -1.6340000000e-03 * w * RA ...
+ 4.7100000000e-04 * LE * RE ...
+ 8.9652000000e-02 * u^2 ...
+ 5.1995000000e-02 * w^2 ...
+ -8.6100000000e-04 * LA^2 ...
+ 9.7800000000e-04 * RE^2 ...
+ 6.9337000000e-02 * u^2 * w ...
+ 3.9900000000e-03 * u^2 * LA ...
+ 4.5510000000e-03 * u^2 * RA ...
+ 1.6720000000e-03 * u^2 * RE ...
+ -8.3030000000e-03 * u * w^2 ...
+ -7.4630000000e-03 * w^2 * LA ...
+ -5.9850000000e-03 * w^2 * RA ...
+ -1.6070000000e-03 * w * LA^2 ...
+ -6.3200000000e-04 * LE * RE^2 ...
+ -8.5198000000e-02 * w^3 ...
+ -8.2700000000e-04 * LA^3;

Faxial = ...
+ 8.5764400000e+00 ...
+ 1.7169500000e+00 * u ...
+ 2.4458700000e-01 * v ...
+ -7.4444900000e-01 * w ...
+ -6.0992000000e-02 * LA ...
+ 1.5143000000e-02 * RA ...
+ -4.2710000000e-03 * LE ...
+ -1.7603000000e-02 * RE ...
+ -8.0674000000e-02 * RUD ...
+ -1.9111000000e-02 * u * w ...
+ 1.5170000000e-03 * u * LA ...
+ 3.6900000000e-04 * u * RA ...
+ 1.2100000000e-03 * u * RUD ...
+ -4.4980000000e-03 * v * RA ...
+ 3.8210000000e-03 * v * LE ...
+ 8.1250000000e-03 * w * LA ...
+ 5.4100000000e-04 * LA * RA ...
+ -5.0500000000e-04 * LA * RUD ...
+ -4.5600000000e-04 * RA * LE ...
+ -1.3480000000e-03 * RA * RUD ...
+ -1.1370000000e-03 * LE * RE ...
+ -1.6500000000e-04 * LE * RUD ...
+ -7.4900000000e-04 * RE * RUD ...
+ -2.7647300000e-01 * u^2 ...
+ -1.6880800000e-01 * w^2 ...
+ -1.2250000000e-03 * LA^2 ...
+ -1.8100000000e-04 * RA^2 ...
+ 5.2900000000e-04 * LE^2 ...
+ -5.2000000000e-04 * RE^2 ...
+ -1.8040000000e-03 * RUD^2 ...
+ 1.5400000000e-03 * v * RA * LE ...
+ -1.0700000000e-04 * LA * RA * RUD ...
+ 1.1800000000e-04 * LE * RE * RUD ...
+ -3.8210000000e-03 * u^2 * w ...
+ -1.8349000000e-02 * u * w^2 ...
+ 5.1500000000e-04 * u * LA^2 ...
+ 5.4000000000e-04 * u * RA^2 ...
+ 3.1300000000e-04 * u * RUD^2 ...
+ 2.9600000000e-04 * LE^2 * RUD ...
+ -1.4500000000e-04 * RE^2 * RUD ...
+ 7.2070000000e-03 * u^3 ...
+ 1.0241000000e-02 * w^3 ...
+ 1.2500000000e-04 * LA^3;

Mpitch = ...
+ 9.2036000000e+00 ...
+ 2.7974500000e+00 * u ...
+ 8.1880400000e-01 * v ...
+ -4.9549300000e+00 * w ...
+ 1.7158300000e-01 * LA ...
+ 1.8059300000e-01 * RA ...
+ -7.9790100000e-01 * LE ...
+ 2.7907000000e-02 * RE ...
+ -3.4022000000e-02 * RUD ...
+ 9.0524600000e-01 * u * w ...
+ -1.7809000000e-02 * u * LA ...
+ -1.7593000000e-02 * u * RA ...
+ -8.5660000000e-03 * u * LE ...
+ -5.5043000000e-02 * u * RE ...
+ 8.6470000000e-03 * u * RUD ...
+ 6.4720000000e-02 * v * LA ...
+ 7.6480000000e-03 * v * RE ...
+ -1.1105000000e-02 * v * RUD ...
+ 1.2870000000e-03 * w * LE ...
+ 3.0050000000e-03 * w * RE ...
+ -5.2000000000e-04 * w * RUD ...
+ -3.6560000000e-03 * LA * RA ...
+ 3.6050000000e-03 * LA * LE ...
+ 2.9970000000e-03 * LA * RUD ...
+ -1.7700000000e-04 * RA * LE ...
+ 1.6130000000e-03 * RA * RUD ...
+ -2.1370000000e-03 * LE * RE ...
+ -2.5580000000e-03 * LE * RUD ...
+ -7.1870000000e-03 * RE * RUD ...
+ -5.5134100000e-01 * u^2 ...
+ -5.1052000000e-02 * v^2 ...
+ -1.7062500000e-01 * w^2 ...
+ 1.0304000000e-02 * LA^2 ...
+ 4.6000000000e-05 * RA^2 ...
+ 3.2760000000e-03 * LE^2 ...
+ -8.7590000000e-03 * RE^2 ...
+ -1.3270000000e-03 * RUD^2 ...
+ -4.9700000000e-04 * u * LA * RUD ...
+ -5.2510000000e-03 * v * RE * RUD ...
+ 1.6040000000e-03 * w * LE * RUD ...
+ -7.8000000000e-04 * w * RE * RUD ...
+ -4.1000000000e-04 * LA * RA * RUD ...
+ 3.8100000000e-04 * LE * RE * RUD ...
+ -8.9490000000e-03 * u^2 * LE ...
+ -6.5160000000e-03 * u^2 * RE ...
+ -4.9880000000e-02 * u * w^2 ...
+ 1.4650000000e-03 * u * RA^2 ...
+ 1.0300000000e-03 * u * RUD^2 ...
+ 1.5082200000e-01 * v^2 * LA ...
+ -1.4976700000e-01 * v^2 * RE ...
+ 8.2620000000e-03 * w^2 * LE ...
+ 4.3910000000e-03 * w^2 * RE ...
+ 1.5940000000e-03 * w * LE^2 ...
+ 3.1940000000e-03 * w * RE^2 ...
+ -5.4700000000e-04 * LA * LE^2 ...
+ -4.0800000000e-04 * LA * RUD^2 ...
+ 5.1200000000e-04 * RA^2 * LE ...
+ -3.5800000000e-04 * RA * RUD^2 ...
+ 1.4315000000e-02 * u^3 ...
+ -1.7379600000e-01 * w^3;

Fside = ...
+ 1.6118900000e-01 ...
+ 1.5867000000e-02 * u ...
+ -3.4130500000e+00 * v ...
+ -1.8110000000e-03 * w ...
+ -6.2800000000e-03 * LA ...
+ 1.6002000000e-02 * RA ...
+ 1.8794000000e-02 * LE ...
+ -1.3204000000e-02 * RE ...
+ 4.3566000000e-02 * RUD ...
+ -7.5175000000e-02 * u * v ...
+ -1.2200000000e-04 * u * w ...
+ 3.4840000000e-03 * u * LA ...
+ -3.2910000000e-03 * u * RA ...
+ -2.5130000000e-03 * u * LE ...
+ 2.2400000000e-03 * u * RE ...
+ -7.7460000000e-03 * u * RUD ...
+ -9.3649000000e-02 * v * w ...
+ 1.8010000000e-03 * v * LA ...
+ -2.7600000000e-04 * w * LA ...
+ 1.2300000000e-03 * w * RA ...
+ 8.3400000000e-04 * w * LE ...
+ 1.0960000000e-03 * w * RE ...
+ 4.3660000000e-03 * w * RUD ...
+ -7.7000000000e-05 * LA * RA ...
+ 6.8000000000e-05 * LA * RE ...
+ 1.2300000000e-04 * LA * RUD ...
+ -7.6300000000e-04 * RA * LE ...
+ -4.2700000000e-04 * RA * RE ...
+ -3.2900000000e-04 * RA * RUD ...
+ -9.6000000000e-05 * LE * RE ...
+ -5.4000000000e-05 * LE * RUD ...
+ 1.2300000000e-04 * RE * RUD ...
+ -8.6800000000e-04 * u^2 ...
+ -1.5125500000e-01 * v^2 ...
+ 6.6480000000e-03 * w^2 ...
+ 6.5000000000e-05 * LA^2 ...
+ -2.2400000000e-04 * RA^2 ...
+ 1.3100000000e-04 * LE^2 ...
+ -1.7900000000e-04 * RE^2 ...
+ 3.2000000000e-05 * RUD^2 ...
+ -9.0590000000e-03 * u * v * w ...
+ -7.9400000000e-04 * u * v * LA ...
+ 2.1200000000e-04 * u * w * LE ...
+ -2.5600000000e-04 * u * w * RE ...
+ 6.4800000000e-04 * u * w * RUD ...
+ 4.9000000000e-05 * u * RA * RE ...
+ -7.4000000000e-05 * u * LE * RUD ...
+ -5.4000000000e-05 * u * RE * RUD ...
+ -7.9000000000e-05 * w * LE * RE ...
+ 8.0000000000e-05 * w * RE * RUD ...
+ 3.2000000000e-05 * LA * RA * RUD ...
+ -3.1000000000e-05 * RA * RE * RUD ...
+ 3.9910000000e-03 * u^2 * v ...
+ -2.6600000000e-04 * u^2 * LA ...
+ 2.9400000000e-04 * u^2 * RA ...
+ -1.0620000000e-03 * u^2 * RUD ...
+ 6.7100000000e-04 * w^2 * LA ...
+ -9.3300000000e-04 * w^2 * RA ...
+ 3.8100000000e-04 * w^2 * LE ...
+ 4.4000000000e-05 * LA^2 * RA ...
+ -4.1000000000e-05 * LA * RE^2 ...
+ 2.2000000000e-05 * RA^2 * RE ...
+ -2.8218200000e-01 * v^3 ...
+ -7.0000000000e-05 * LE^3 ...
+ -8.0000000000e-05 * RUD^3;

Mroll = ...
+ 6.1645300000e-01 ...
+ -5.1636000000e-02 * u ...
+ 1.2545000000e+00 * v ...
+ 1.7016100000e-01 * w ...
+ 6.9748000000e+00 * LA ...
+ -6.2734100000e+00 * RA ...
+ 2.0742200000e-01 * LE ...
+ -2.0081600000e-01 * RE ...
+ 9.3396000000e-02 * RUD ...
+ 7.5498000000e-02 * u * LA ...
+ -5.9276000000e-02 * u * RA ...
+ -5.7566000000e-02 * u * RUD ...
+ -5.7199600000e+00 * v * w ...
+ -3.0230400000e-01 * v * LA ...
+ 2.0138000000e-01 * v * RA ...
+ -8.4614000000e-02 * v * LE ...
+ 1.1607600000e-01 * v * RE ...
+ -3.5875100000e-01 * v * RUD ...
+ -1.6733000000e-02 * w * LA ...
+ -1.7045000000e-02 * w * RA ...
+ -1.8462000000e-02 * LA * LE ...
+ -5.5100000000e-04 * RA * LE ...
+ -1.2360000000e-03 * RA * RUD ...
+ 1.5952000000e-02 * RE * RUD ...
+ -7.9180000000e-03 * u^2 ...
+ 8.0055500000e+00 * v^2 ...
+ -3.1940000000e-03 * w^2 ...
+ -7.5830000000e-03 * LA^2 ...
+ -1.2252000000e-02 * RA^2 ...
+ -2.2243000000e-02 * LE^2 ...
+ -5.2500000000e-04 * RUD^2 ...
+ 2.4229000000e-02 * v * LA * LE ...
+ -2.4147000000e-02 * v * RA * LE ...
+ -2.2018000000e-02 * v * RA * RUD ...
+ 5.5069000000e-02 * u^2 * LA ...
+ -5.8129000000e-02 * u^2 * RA ...
+ -4.6930000000e-03 * u^2 * RUD ...
+ -6.1151000000e-01 * v^2 * RE ...
+ -8.7520000000e-02 * w^2 * LA ...
+ 1.0365000000e-01 * w^2 * RA ...
+ -1.8734000000e-02 * w * LA^2 ...
+ 1.6190000000e-02 * w * RA^2 ...
+ -2.2800000000e-03 * LA * LE^2 ...
+ 2.4540000000e-03 * RE * RUD^2;

Myaw = ...
+ -2.8242400000e+00 ...
+ -9.9510000000e-03 * u ...
+ 3.0052880000e+01 * v ...
+ -7.5120000000e-02 * w ...
+ -2.0074800000e-01 * LA ...
+ 6.6463200000e-01 * RA ...
+ 8.2045000000e-02 * LE ...
+ 9.8510000000e-02 * RE ...
+ -8.4310000000e-03 * RUD ...
+ 8.2144000000e-02 * u * v ...
+ -2.1668000000e-02 * u * w ...
+ 9.1520000000e-03 * u * LA ...
+ -2.7734000000e-02 * u * RA ...
+ 3.0978000000e-02 * u * LE ...
+ -2.0650000000e-02 * u * RE ...
+ 1.1149900000e-01 * u * RUD ...
+ -1.0174400000e+00 * v * w ...
+ -5.9620000000e-02 * v * LA ...
+ -7.9453000000e-02 * v * RA ...
+ 4.8784000000e-02 * v * LE ...
+ -4.2484000000e-02 * v * RE ...
+ -1.4749000000e-02 * v * RUD ...
+ -1.2367100000e-01 * w * LA ...
+ 1.3029600000e-01 * w * RA ...
+ 5.6400000000e-04 * w * LE ...
+ -7.6000000000e-04 * w * RE ...
+ -3.7620000000e-02 * w * RUD ...
+ -9.0900000000e-04 * RA * LE ...
+ 1.9150000000e-03 * RA * RUD ...
+ 6.0390000000e-03 * LE * RE ...
+ 8.1380000000e-03 * LE * RUD ...
+ 4.5430000000e-03 * RE * RUD ...
+ 3.3400000000e-02 * u^2 ...
+ -5.4469700000e-01 * v^2 ...
+ -1.7735000000e-02 * w^2 ...
+ -1.8046000000e-02 * LA^2 ...
+ 2.0271000000e-02 * RA^2 ...
+ 5.8000000000e-05 * RUD^2 ...
+ 1.3366000000e-02 * u * v * LA ...
+ 1.4861000000e-02 * u * v * RUD ...
+ 5.8220000000e-03 * u * w * LA ...
+ -7.1880000000e-03 * u * w * RA ...
+ -3.8160000000e-03 * u * w * LE ...
+ 4.0040000000e-03 * u * w * RE ...
+ -1.2638000000e-02 * u * w * RUD ...
+ 8.0200000000e-04 * u * RE * RUD ...
+ 6.9860000000e-03 * v * RA * LE ...
+ -5.7400000000e-04 * LE * RE * RUD ...
+ -5.0058000000e-02 * u^2 * v ...
+ 1.9017000000e-02 * u^2 * RUD ...
+ -7.0730000000e-03 * u * LA^2 ...
+ 5.6520000000e-03 * u * RA^2 ...
+ 2.3937600000e-01 * v^2 * LE ...
+ -5.0903000000e-02 * v^2 * RE ...
+ -1.0293000000e-02 * v * RUD^2 ...
+ 4.6170000000e-03 * w^2 * RA ...
+ -1.3000000000e-03 * RA * RUD^2 ...
+ -1.0470000000e-03 * LE * RUD^2;

Flift = ...
+ -4.4660600000e+00 ...
+ 6.6353000000e-01 * u ...
+ 1.3820950000e+01 * w ...
+ 1.0923200000e+00 * LA ...
+ 3.9502100000e-01 * RA ...
+ 3.4015700000e-01 * LE ...
+ -7.4771000000e-02 * RE ...
+ 1.1774700000e+00 * u * w ...
+ 4.6640000000e-03 * u * LA ...
+ -7.7870000000e-03 * u * RA ...
+ 2.2103000000e-02 * u * LE ...
+ -1.0929000000e-02 * u * RE ...
+ -9.7350000000e-03 * w * LA ...
+ -2.8460000000e-03 * w * RA ...
+ -2.6400000000e-04 * RA * LE ...
+ -1.6420000000e-03 * RA * RE ...
+ 1.1270000000e-03 * LE * RE ...
+ 6.8750000000e-02 * u^2 ...
+ -2.1860000000e-02 * w^2 ...
+ -4.1900000000e-04 * LA^2 ...
+ 6.6200000000e-04 * RA^2 ...
+ 2.2640000000e-03 * RE^2 ...
+ -2.7600000000e-04 * RA * LE * RE ...
+ 4.7812000000e-02 * u^2 * w ...
+ 3.7160000000e-03 * u^2 * LA ...
+ 4.3840000000e-03 * u^2 * RA ...
+ 1.6930000000e-03 * u^2 * RE ...
+ -9.0790000000e-03 * w^2 * LA ...
+ -7.6400000000e-03 * w^2 * RA ...
+ -1.6620000000e-03 * w * LA^2 ...
+ -1.5920000000e-03 * w * RA^2 ...
+ -6.1700000000e-04 * LE * RE^2 ...
+ -1.2567100000e-01 * w^3 ...
+ -7.5200000000e-04 * LA^3;

Fdrag = ...
+ 2.9328600000e+00 ...
+ -3.6328400000e-01 * u ...
+ -2.4149000000e-02 * v ...
+ 2.9734200000e-01 * w ...
+ -3.0471000000e-02 * LA ...
+ -1.0251000000e-02 * RA ...
+ -7.3030000000e-03 * LE ...
+ 3.1160000000e-03 * RE ...
+ 9.4150000000e-03 * RUD ...
+ 8.3390000000e-03 * u * v ...
+ -3.7260000000e-02 * u * w ...
+ 6.9800000000e-04 * u * LA ...
+ 1.1740000000e-03 * u * RA ...
+ 2.4200000000e-04 * u * LE ...
+ 1.8800000000e-04 * u * RE ...
+ 4.7000000000e-05 * u * RUD ...
+ -2.3190000000e-03 * v * w ...
+ -2.7840000000e-03 * v * LA ...
+ 3.5930000000e-03 * v * RUD ...
+ 9.4110000000e-03 * w * LA ...
+ 9.7870000000e-03 * w * RA ...
+ -1.4000000000e-04 * w * LE ...
+ 1.8970000000e-03 * w * RE ...
+ 1.6090000000e-03 * w * RUD ...
+ -9.3000000000e-04 * LE * RE ...
+ -1.6100000000e-04 * LE * RUD ...
+ -9.9000000000e-05 * RE * RUD ...
+ 6.1270000000e-02 * u^2 ...
+ 6.6310300000e-01 * v^2 ...
+ 1.2978900000e+00 * w^2 ...
+ -6.9200000000e-04 * LA^2 ...
+ -2.6600000000e-04 * RA^2 ...
+ 6.5900000000e-04 * LE^2 ...
+ 4.1600000000e-04 * RUD^2 ...
+ 1.2040000000e-03 * u * v * RUD ...
+ 2.7690000000e-03 * u * w * LA ...
+ 2.7490000000e-03 * u * w * RA ...
+ 9.9700000000e-04 * u * w * LE ...
+ 9.5000000000e-04 * u * w * RE ...
+ -1.5870000000e-03 * v * w * LA ...
+ 1.3300000000e-04 * w * LE * RUD ...
+ 3.7000000000e-05 * LE * RE * RUD ...
+ 5.2780000000e-03 * u * w^2 ...
+ 2.1500000000e-04 * u * LA^2 ...
+ 2.8500000000e-04 * u * RA^2 ...
+ 1.8700000000e-04 * u * RUD^2 ...
+ -1.7600000000e-04 * w * LE^2 ...
+ -8.3500000000e-04 * u^3 ...
+ 2.6990000000e-03 * w^3 ...
+ 5.3000000000e-05 * LA^3;

% Aerodynamic forces [lbf] and moments [ft-lbf]
AeroModel = [Fnormal Faxial Mpitch Fside Mroll Myaw Flift Fdrag];

return
