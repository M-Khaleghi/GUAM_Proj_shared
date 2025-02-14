function [C,Metrics]=LpC_Hover_R7t10_PCp2FI_MOF_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8)
% LpC_Hover_R7t10_PCp2FI_MOF_cv - Aerodynamic model for the LpC Hover
%
% DESCRIPTION: 
%   This script contains the aerodynamic model for LpC Hover. The script was
%   automatically generated using "GenModelCV.m" on 16-Mar-2021 13:42:14
%
% INPUTS:
%   List of column vectors containing the model explanatory variables 
%       The variables are in the following order:
%       [u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8]
%       Units: deg  kts  rpm 
%
% OUTPUTS:
%   C - Matrix containing the model response variables in each column 
%       The variables are in the following order:
%       [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag]
%       Units: ft-lbf  lbf 
%   Metrics - Modeling metrics (R^2, PSE, PRESS, and number of terms) 
%
% CALLS:
%   None
%
% WRITTEN BY:
%   Benjamin M. Simmons
%   NASA Langley Research Center
%   Email: benjamin.m.simmons@nasa.gov
%
% HISTORY:
%   February 25, 2020 - created and debugged, BMS
%   May 08, 2020 - update for LA-8 aero model release v1.0, BMS
%   June 11, 2020 - update for RAM results, BMS
%
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
% MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
% IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
% CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
% TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
% SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
%
% ***FOR INTERNAL NASA LARC USE ONLY***
%

%   Modeling Algorithm: MOF, Model Complexity: Quadratic+2FI+PC 
%   Model SF: 0.2, SWR Alpha: 1 
%   Final Fval: [  0  0  0  0  0  0  0  0  ] 
%
%   MSD Explanatory Variables for each Response Variable: 
%    Faxial: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv
%    Fside: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv
%    Fnormal: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv
%    Mroll: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv
%    Mpitch: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv
%    Myaw: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv
%    Flift: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv
%    Fdrag: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv
%    
%   Files used to develop model:
%    BlockL2_R1_S1_B1_D2_output.dat  BlockL2_R1_S1_B2_D3_output.dat  BlockL2_R1_S1_B3_D4_output.dat  BlockL2_R1_S1_B4_D5_output.dat 
%
%   Explantory variable ranges used to develop model:
%             u: [      -5,     +20] kts,  Nord:  3
%             v: [     -10,     +10] kts,  Nord:  3
%             w: [     -10,     +10] kts,  Nord:  3
%            LA: [     -30,     +30] deg,  Nord:  3
%            RA: [     -30,     +30] deg,  Nord:  3
%            LE: [     -30,     +30] deg,  Nord:  3
%            RE: [     -30,     +30] deg,  Nord:  3
%           RUD: [     -30,     +30] deg,  Nord:  3
%            N1: [    +550,   +1550] rpm,  Nord:  3
%            N2: [    +550,   +1550] rpm,  Nord:  3
%            N3: [    +550,   +1550] rpm,  Nord:  3
%            N4: [    +550,   +1550] rpm,  Nord:  3
%            N5: [    +550,   +1550] rpm,  Nord:  3
%            N6: [    +550,   +1550] rpm,  Nord:  3
%            N7: [    +550,   +1550] rpm,  Nord:  3
%            N8: [    +550,   +1550] rpm,  Nord:  3
%
%   This model was developed using Rapid Aero Modeling (RAM) software.
%   Programs from the System IDentification Programs for AirCraft, or
%   SIDPAC, MATLAB software toolbox were used to determine the model
%   structure and parameter estimates shown below.
%
%
%   Rapid Aero Modeling (RAM) References:
%   [1] Murphy, P. C., Simmons, B. M., Hatke, D. B., and Busan, R. C.,
%       "Rapid Aero Modeling for Urban Air Mobility Aircraft in Wind-Tunnel
%       Tests," AIAA SciTech 2021 Forum, AIAA Paper 2021-1644.
%
%   [2] Murphy, P. C., Buning, P. G., and Simmons, B. M., "Rapid Aero
%       Modeling for Urban Air Mobility Aircraft in Computational
%       Experiments," AIAA SciTech 2021 Forum, AIAA Paper 2021-1002.
%


%Initialize modeling metrics
RSQ=zeros(1,8);
PSE=zeros(1,8);
PRESS=zeros(1,8);
NTERMS=zeros(1,8);


% Conversion from natural variables to coded variables
u_cv = ( u-(7.50000000000000E+00) )/(1.25000000000000E+01);
v_cv = ( v-(0.00000000000000E+00) )/(1.00000000000000E+01);
w_cv = ( w-(0.00000000000000E+00) )/(1.00000000000000E+01);
LA_cv = ( LA-(0.00000000000000E+00) )/(3.00000000000000E+01);
RA_cv = ( RA-(0.00000000000000E+00) )/(3.00000000000000E+01);
LE_cv = ( LE-(0.00000000000000E+00) )/(3.00000000000000E+01);
RE_cv = ( RE-(0.00000000000000E+00) )/(3.00000000000000E+01);
RUD_cv = ( RUD-(0.00000000000000E+00) )/(3.00000000000000E+01);
N1_cv = ( N1-(1.05000000000000E+03) )/(5.00000000000000E+02);
N2_cv = ( N2-(1.05000000000000E+03) )/(5.00000000000000E+02);
N3_cv = ( N3-(1.05000000000000E+03) )/(5.00000000000000E+02);
N4_cv = ( N4-(1.05000000000000E+03) )/(5.00000000000000E+02);
N5_cv = ( N5-(1.05000000000000E+03) )/(5.00000000000000E+02);
N6_cv = ( N6-(1.05000000000000E+03) )/(5.00000000000000E+02);
N7_cv = ( N7-(1.05000000000000E+03) )/(5.00000000000000E+02);
N8_cv = ( N8-(1.05000000000000E+03) )/(5.00000000000000E+02);




%Response variable polynomials

% Faxial Model
RSQ(1)=96.637; PSE(1)=1876.1; PRESS(1)=1.1431E+06; NTERMS(1)=46;
Faxial = ...
  +2.10823306491492E+02 .*                 u_cv  + ... % SE: 6.7306E+00, PE:   3.19%, 95% CI: [+1.9736E+02,+2.2428E+02]
  -1.98041224251710E+01 .*              w_cv.^3  + ... % SE: 7.3143E+00, PE:  36.93%, 95% CI: [-3.4433E+01,-5.1755E+00]
  +6.79962797327550E+01 .*                    1  + ... % SE: 2.8021E+00, PE:   4.12%, 95% CI: [+6.2392E+01,+7.3600E+01]
  -6.43905977462200E+01 .*              u_cv.^2  + ... % SE: 4.3812E+00, PE:   6.80%, 95% CI: [-7.3153E+01,-5.5628E+01]
  -3.54253522508067E+01 .*         u_cv .* w_cv  + ... % SE: 2.1605E+00, PE:   6.10%, 95% CI: [-3.9746E+01,-3.1104E+01]
  +2.80956191933570E+01 .*        u_cv .* N1_cv  + ... % SE: 2.1806E+00, PE:   7.76%, 95% CI: [+2.3734E+01,+3.2457E+01]
  +2.57614280498660E+01 .*        u_cv .* N4_cv  + ... % SE: 2.1872E+00, PE:   8.49%, 95% CI: [+2.1387E+01,+3.0136E+01]
  +2.56939683119349E+01 .*        u_cv .* N5_cv  + ... % SE: 2.1948E+00, PE:   8.54%, 95% CI: [+2.1304E+01,+3.0083E+01]
  +2.38140865710684E+01 .*        u_cv .* N7_cv  + ... % SE: 2.1797E+00, PE:   9.15%, 95% CI: [+1.9455E+01,+2.8173E+01]
  +2.31629426798442E+01 .*        u_cv .* N3_cv  + ... % SE: 2.1882E+00, PE:   9.45%, 95% CI: [+1.8787E+01,+2.7539E+01]
  +2.18466860540786E+01 .*        u_cv .* N8_cv  + ... % SE: 2.1783E+00, PE:   9.97%, 95% CI: [+1.7490E+01,+2.6203E+01]
  +2.27750714650054E+01 .*        u_cv .* N6_cv  + ... % SE: 2.1858E+00, PE:   9.60%, 95% CI: [+1.8403E+01,+2.7147E+01]
  +2.12827647253664E+01 .*        u_cv .* N2_cv  + ... % SE: 2.1846E+00, PE:  10.26%, 95% CI: [+1.6914E+01,+2.5652E+01]
  -1.48357386089749E+01 .*                N3_cv  + ... % SE: 1.8857E+00, PE:  12.71%, 95% CI: [-1.8607E+01,-1.1064E+01]
  +1.52190848625219E+01 .*        v_cv .* N6_cv  + ... % SE: 2.1848E+00, PE:  14.36%, 95% CI: [+1.0849E+01,+1.9589E+01]
  -1.26855918849832E+01 .*                N5_cv  + ... % SE: 1.9110E+00, PE:  15.06%, 95% CI: [-1.6508E+01,-8.8635E+00]
  +1.18853504137565E+01 .*             N1_cv.^3  + ... % SE: 2.0388E+00, PE:  17.15%, 95% CI: [+7.8077E+00,+1.5963E+01]
  +1.00575365748498E+01 .*                N8_cv  + ... % SE: 1.8914E+00, PE:  18.81%, 95% CI: [+6.2748E+00,+1.3840E+01]
  +9.81352742203413E+00 .*                N2_cv  + ... % SE: 1.8946E+00, PE:  19.31%, 95% CI: [+6.0243E+00,+1.3603E+01]
  +1.06305275553554E+01 .*        v_cv .* N7_cv  + ... % SE: 2.1820E+00, PE:  20.53%, 95% CI: [+6.2665E+00,+1.4995E+01]
  -1.08087666243593E+01 .*        v_cv .* N1_cv  + ... % SE: 2.1786E+00, PE:  20.16%, 95% CI: [-1.5166E+01,-6.4516E+00]
  -2.64872462850812E+01 .*              w_cv.^2  + ... % SE: 4.3769E+00, PE:  16.52%, 95% CI: [-3.5241E+01,-1.7733E+01]
  +3.07852374250541E+01 .*              u_cv.^3  + ... % SE: 7.2788E+00, PE:  23.64%, 95% CI: [+1.6228E+01,+4.5343E+01]
  +9.17215324579437E+00 .*       LE_cv .* N8_cv  + ... % SE: 2.1874E+00, PE:  23.85%, 95% CI: [+4.7974E+00,+1.3547E+01]
  -8.52867629954997E+00 .*        v_cv .* N5_cv  + ... % SE: 2.2044E+00, PE:  25.85%, 95% CI: [-1.2938E+01,-4.1198E+00]
  +7.90691009883611E+00 .*                N7_cv  + ... % SE: 1.8922E+00, PE:  23.93%, 95% CI: [+4.1226E+00,+1.1691E+01]
  -9.01297378332419E+00 .*        v_cv .* N4_cv  + ... % SE: 2.1958E+00, PE:  24.36%, 95% CI: [-1.3405E+01,-4.6214E+00]
  +7.46927245347220E+00 .*                N4_cv  + ... % SE: 1.9006E+00, PE:  25.45%, 95% CI: [+3.6682E+00,+1.1270E+01]
  +1.67332202781870E+01 .*             N7_cv.^2  + ... % SE: 4.3670E+00, PE:  26.10%, 95% CI: [+7.9993E+00,+2.5467E+01]
  +7.22044470354448E+00 .*       RA_cv .* N8_cv  + ... % SE: 2.1919E+00, PE:  30.36%, 95% CI: [+2.8367E+00,+1.1604E+01]
  +6.67963581192315E+00 .*       N1_cv .* N3_cv  + ... % SE: 2.1937E+00, PE:  32.84%, 95% CI: [+2.2922E+00,+1.1067E+01]
  -7.31024018766414E+00 .*        v_cv .* N8_cv  + ... % SE: 2.1913E+00, PE:  29.98%, 95% CI: [-1.1693E+01,-2.9276E+00]
  +6.34557817314769E+00 .*                N6_cv  + ... % SE: 1.8918E+00, PE:  29.81%, 95% CI: [+2.5621E+00,+1.0129E+01]
  -7.00731601328847E+00 .*       RA_cv .* N2_cv  + ... % SE: 2.1831E+00, PE:  31.15%, 95% CI: [-1.1373E+01,-2.6412E+00]
  +6.63455070201990E+00 .*        u_cv .* LA_cv  + ... % SE: 2.1699E+00, PE:  32.71%, 95% CI: [+2.2947E+00,+1.0974E+01]
  +6.45794599418192E+00 .*        v_cv .* N2_cv  + ... % SE: 2.1885E+00, PE:  33.89%, 95% CI: [+2.0810E+00,+1.0835E+01]
  -7.06108383350160E+00 .*      LA_cv .* RUD_cv  + ... % SE: 2.1961E+00, PE:  31.10%, 95% CI: [-1.1453E+01,-2.6688E+00]
  -5.54659206009092E+00 .*        w_cv .* N2_cv  + ... % SE: 2.1703E+00, PE:  39.13%, 95% CI: [-9.8872E+00,-1.2060E+00]
  +6.19251627706874E+00 .*        w_cv .* N3_cv  + ... % SE: 2.1656E+00, PE:  34.97%, 95% CI: [+1.8613E+00,+1.0524E+01]
  +5.64880056153838E+00 .*       LA_cv .* N2_cv  + ... % SE: 2.2017E+00, PE:  38.98%, 95% CI: [+1.2453E+00,+1.0052E+01]
  -1.77675525276426E+01 .*                 w_cv  + ... % SE: 6.7550E+00, PE:  38.02%, 95% CI: [-3.1278E+01,-4.2575E+00]
  -5.93858839068494E+00 .*        w_cv .* N4_cv  + ... % SE: 2.1869E+00, PE:  36.83%, 95% CI: [-1.0312E+01,-1.5647E+00]
  -5.83786035530722E+00 .*       RE_cv .* N2_cv  + ... % SE: 2.2264E+00, PE:  38.14%, 95% CI: [-1.0291E+01,-1.3851E+00]
  +6.00269555260290E+00 .*       N3_cv .* N7_cv  + ... % SE: 2.1861E+00, PE:  36.42%, 95% CI: [+1.6306E+00,+1.0375E+01]
  +5.78052007495074E+00 .*       N4_cv .* N5_cv  + ... % SE: 2.2363E+00, PE:  38.69%, 95% CI: [+1.3080E+00,+1.0253E+01]
  -5.18257776919213E+00 .*       w_cv .* RUD_cv ;      % SE: 2.1855E+00, PE:  42.17%, 95% CI: [-9.5537E+00,-8.1150E-01]

% Fside Model
RSQ(2)=96.825; PSE(2)=3982.9; PRESS(2)=2.498E+06; NTERMS(2)=29;
Fside = ...
  -2.29804378931284E+02 .*                 v_cv  + ... % SE: 2.8374E+00, PE:   1.23%, 95% CI: [-2.3548E+02,-2.2413E+02]
  -1.79278576594131E+02 .*                N3_cv  + ... % SE: 2.8342E+00, PE:   1.58%, 95% CI: [-1.8495E+02,-1.7361E+02]
  +1.76899580991759E+02 .*                N5_cv  + ... % SE: 2.8779E+00, PE:   1.63%, 95% CI: [+1.7114E+02,+1.8266E+02]
  -1.46158958825973E+02 .*                N4_cv  + ... % SE: 2.8663E+00, PE:   1.96%, 95% CI: [-1.5189E+02,-1.4043E+02]
  +1.45238693192477E+02 .*                N6_cv  + ... % SE: 2.8523E+00, PE:   1.96%, 95% CI: [+1.3953E+02,+1.5094E+02]
  -2.56126220134234E+01 .*         v_cv .* w_cv  + ... % SE: 3.2657E+00, PE:  12.75%, 95% CI: [-3.2144E+01,-1.9081E+01]
  -2.06150827848918E+01 .*        v_cv .* N7_cv  + ... % SE: 3.2921E+00, PE:  15.97%, 95% CI: [-2.7199E+01,-1.4031E+01]
  -1.95574331686958E+01 .*        v_cv .* N2_cv  + ... % SE: 3.2884E+00, PE:  16.81%, 95% CI: [-2.6134E+01,-1.2981E+01]
  -1.28730406860493E+01 .*        v_cv .* N5_cv  + ... % SE: 3.3122E+00, PE:  25.73%, 95% CI: [-1.9497E+01,-6.2486E+00]
  -1.62071794775347E+01 .*        v_cv .* N1_cv  + ... % SE: 3.2747E+00, PE:  20.20%, 95% CI: [-2.2756E+01,-9.6579E+00]
  +1.39208818989658E+01 .*             N7_cv.^3  + ... % SE: 3.0803E+00, PE:  22.13%, 95% CI: [+7.7603E+00,+2.0081E+01]
  +1.40791032566553E+01 .*        u_cv .* N5_cv  + ... % SE: 3.3043E+00, PE:  23.47%, 95% CI: [+7.4705E+00,+2.0688E+01]
  +1.21630853831122E+01 .*        u_cv .* N4_cv  + ... % SE: 3.2870E+00, PE:  27.02%, 95% CI: [+5.5891E+00,+1.8737E+01]
  -1.24701142437534E+01 .*        u_cv .* N6_cv  + ... % SE: 3.2915E+00, PE:  26.40%, 95% CI: [-1.9053E+01,-5.8871E+00]
  -1.41139192157077E+01 .*        v_cv .* N3_cv  + ... % SE: 3.2791E+00, PE:  23.23%, 95% CI: [-2.0672E+01,-7.5558E+00]
  -1.26213450176477E+01 .*        v_cv .* N6_cv  + ... % SE: 3.2931E+00, PE:  26.09%, 95% CI: [-1.9208E+01,-6.0351E+00]
  -1.13121404188242E+01 .*       LA_cv .* N4_cv  + ... % SE: 3.3050E+00, PE:  29.22%, 95% CI: [-1.7922E+01,-4.7022E+00]
  +9.85685868924386E+00 .*        w_cv .* N8_cv  + ... % SE: 3.2912E+00, PE:  33.39%, 95% CI: [+3.2744E+00,+1.6439E+01]
  -1.04040564059131E+01 .*                N1_cv  + ... % SE: 2.8401E+00, PE:  27.30%, 95% CI: [-1.6084E+01,-4.7239E+00]
  -1.22378022625547E+01 .*        u_cv .* N3_cv  + ... % SE: 3.2729E+00, PE:  26.74%, 95% CI: [-1.8784E+01,-5.6920E+00]
  -1.12037401273476E+01 .*        v_cv .* N8_cv  + ... % SE: 3.3015E+00, PE:  29.47%, 95% CI: [-1.7807E+01,-4.6008E+00]
  -1.11962803314708E+01 .*        v_cv .* N4_cv  + ... % SE: 3.3039E+00, PE:  29.51%, 95% CI: [-1.7804E+01,-4.5886E+00]
  +4.37812558290196E+01 .*             N5_cv.^2  + ... % SE: 6.5232E+00, PE:  14.90%, 95% CI: [+3.0735E+01,+5.6828E+01]
  -4.86243824775947E+01 .*             N3_cv.^2  + ... % SE: 6.6784E+00, PE:  13.73%, 95% CI: [-6.1981E+01,-3.5268E+01]
  +3.80237863692897E+01 .*             N6_cv.^2  + ... % SE: 6.6457E+00, PE:  17.48%, 95% CI: [+2.4732E+01,+5.1315E+01]
  -3.00634018797724E+01 .*             N4_cv.^2  + ... % SE: 6.5873E+00, PE:  21.91%, 95% CI: [-4.3238E+01,-1.6889E+01]
  -9.85515176406886E+00 .*        w_cv .* N2_cv  + ... % SE: 3.2764E+00, PE:  33.25%, 95% CI: [-1.6408E+01,-3.3023E+00]
  +9.67880701998893E+00 .*        w_cv .* LA_cv  + ... % SE: 3.2800E+00, PE:  33.89%, 95% CI: [+3.1188E+00,+1.6239E+01]
  +9.08413730063057E+00 .*        w_cv .* N6_cv ;      % SE: 3.2852E+00, PE:  36.16%, 95% CI: [+2.5137E+00,+1.5655E+01]

% Fnormal Model
RSQ(3)=98.844; PSE(3)=83253; PRESS(3)=3.8658E+07; NTERMS(3)=28;
Fnormal = ...
  +7.88982590522174E+03 .*                    1  + ... % SE: 1.7523E+01, PE:   0.22%, 95% CI: [+7.8548E+03,+7.9249E+03]
  +9.90575531948088E+02 .*                N8_cv  + ... % SE: 1.1270E+01, PE:   1.14%, 95% CI: [+9.6804E+02,+1.0131E+03]
  +9.84313212751424E+02 .*                N2_cv  + ... % SE: 1.1280E+01, PE:   1.15%, 95% CI: [+9.6175E+02,+1.0069E+03]
  +9.50589132308654E+02 .*                N4_cv  + ... % SE: 1.1299E+01, PE:   1.19%, 95% CI: [+9.2799E+02,+9.7319E+02]
  +9.57168360418181E+02 .*                N6_cv  + ... % SE: 1.1260E+01, PE:   1.18%, 95% CI: [+9.3465E+02,+9.7969E+02]
  +8.66548468301120E+02 .*                N7_cv  + ... % SE: 1.1239E+01, PE:   1.30%, 95% CI: [+8.4407E+02,+8.8903E+02]
  +8.42184405642729E+02 .*                N1_cv  + ... % SE: 1.1215E+01, PE:   1.33%, 95% CI: [+8.1975E+02,+8.6461E+02]
  +7.92934857460170E+02 .*                N5_cv  + ... % SE: 1.1351E+01, PE:   1.43%, 95% CI: [+7.7023E+02,+8.1564E+02]
  +7.82058542231818E+02 .*                N3_cv  + ... % SE: 1.1220E+01, PE:   1.43%, 95% CI: [+7.5962E+02,+8.0450E+02]
  +2.37438945746033E+02 .*             N8_cv.^2  + ... % SE: 2.8318E+01, PE:  11.93%, 95% CI: [+1.8080E+02,+2.9407E+02]
  +4.92883930603975E+02 .*                 w_cv  + ... % SE: 1.1155E+01, PE:   2.26%, 95% CI: [+4.7057E+02,+5.1519E+02]
  +2.49168047863946E+02 .*             N2_cv.^2  + ... % SE: 2.8014E+01, PE:  11.24%, 95% CI: [+1.9314E+02,+3.0520E+02]
  +2.87503466451006E+02 .*         u_cv .* w_cv  + ... % SE: 1.2870E+01, PE:   4.48%, 95% CI: [+2.6176E+02,+3.1324E+02]
  +2.12046467879846E+02 .*             N7_cv.^2  + ... % SE: 2.8392E+01, PE:  13.39%, 95% CI: [+1.5526E+02,+2.6883E+02]
  +1.77024745336275E+02 .*                 u_cv  + ... % SE: 1.1168E+01, PE:   6.31%, 95% CI: [+1.5469E+02,+1.9936E+02]
  +2.04656173764728E+02 .*             N1_cv.^2  + ... % SE: 2.8623E+01, PE:  13.99%, 95% CI: [+1.4741E+02,+2.6190E+02]
  +1.92089638102888E+02 .*             N6_cv.^2  + ... % SE: 2.8353E+01, PE:  14.76%, 95% CI: [+1.3538E+02,+2.4880E+02]
  -1.21629374326660E+02 .*        u_cv .* N5_cv  + ... % SE: 1.3057E+01, PE:  10.74%, 95% CI: [-1.4774E+02,-9.5515E+01]
  +1.17361016823027E+02 .*        u_cv .* N6_cv  + ... % SE: 1.2952E+01, PE:  11.04%, 95% CI: [+9.1457E+01,+1.4326E+02]
  +1.75742935512682E+02 .*             N3_cv.^2  + ... % SE: 2.8427E+01, PE:  16.18%, 95% CI: [+1.1889E+02,+2.3260E+02]
  +1.01329407509973E+02 .*        u_cv .* N2_cv  + ... % SE: 1.2994E+01, PE:  12.82%, 95% CI: [+7.5342E+01,+1.2732E+02]
  -9.48055036989403E+01 .*        u_cv .* N3_cv  + ... % SE: 1.2917E+01, PE:  13.62%, 95% CI: [-1.2064E+02,-6.8972E+01]
  +9.60635810916595E+01 .*        u_cv .* N4_cv  + ... % SE: 1.2971E+01, PE:  13.50%, 95% CI: [+7.0121E+01,+1.2201E+02]
  +1.98761902862117E+02 .*             N5_cv.^2  + ... % SE: 2.8007E+01, PE:  14.09%, 95% CI: [+1.4275E+02,+2.5478E+02]
  -8.16551999905106E+01 .*        u_cv .* N1_cv  + ... % SE: 1.2906E+01, PE:  15.81%, 95% CI: [-1.0747E+02,-5.5843E+01]
  -8.02109495733266E+01 .*        u_cv .* N7_cv  + ... % SE: 1.2938E+01, PE:  16.13%, 95% CI: [-1.0609E+02,-5.4335E+01]
  +1.75485342732309E+02 .*             N4_cv.^2  + ... % SE: 2.8107E+01, PE:  16.02%, 95% CI: [+1.1927E+02,+2.3170E+02]
  +7.82971060976921E+01 .*        u_cv .* N8_cv ;      % SE: 1.2970E+01, PE:  16.56%, 95% CI: [+5.2357E+01,+1.0424E+02]

% Mroll Model
RSQ(4)=99.108; PSE(4)=1.2309E+07; PRESS(4)=5.4767E+09; NTERMS(4)=23;
Mroll = ...
  -1.81551143110081E+04 .*                N8_cv  + ... % SE: 1.3460E+02, PE:   0.74%, 95% CI: [-1.8424E+04,-1.7886E+04]
  +1.81762695135536E+04 .*                N2_cv  + ... % SE: 1.3456E+02, PE:   0.74%, 95% CI: [+1.7907E+04,+1.8445E+04]
  +1.58248031315097E+04 .*                N1_cv  + ... % SE: 1.3372E+02, PE:   0.84%, 95% CI: [+1.5557E+04,+1.6092E+04]
  -1.57526884612839E+04 .*                N7_cv  + ... % SE: 1.3421E+02, PE:   0.85%, 95% CI: [-1.6021E+04,-1.5484E+04]
  -7.21307658714940E+03 .*                N6_cv  + ... % SE: 1.3445E+02, PE:   1.86%, 95% CI: [-7.4820E+03,-6.9442E+03]
  +7.21392899244942E+03 .*                N4_cv  + ... % SE: 1.3483E+02, PE:   1.87%, 95% CI: [+6.9443E+03,+7.4836E+03]
  +5.97548293814316E+03 .*                N3_cv  + ... % SE: 1.3389E+02, PE:   2.24%, 95% CI: [+5.7077E+03,+6.2433E+03]
  -5.94274864547679E+03 .*                N5_cv  + ... % SE: 1.3555E+02, PE:   2.28%, 95% CI: [-6.2138E+03,-5.6716E+03]
  -3.76660831440257E+03 .*                 v_cv  + ... % SE: 1.3381E+02, PE:   3.55%, 95% CI: [-4.0342E+03,-3.4990E+03]
  -3.90125654751963E+03 .*         v_cv .* w_cv  + ... % SE: 1.5428E+02, PE:   3.95%, 95% CI: [-4.2098E+03,-3.5927E+03]
  +1.75846038083457E+03 .*        u_cv .* N7_cv  + ... % SE: 1.5441E+02, PE:   8.78%, 95% CI: [+1.4496E+03,+2.0673E+03]
  -1.75871448222226E+03 .*        u_cv .* N1_cv  + ... % SE: 1.5409E+02, PE:   8.76%, 95% CI: [-2.0669E+03,-1.4505E+03]
  +1.53043131083598E+03 .*         u_cv .* v_cv  + ... % SE: 1.5403E+02, PE:  10.06%, 95% CI: [+1.2224E+03,+1.8385E+03]
  -1.35573428405154E+03 .*        u_cv .* N8_cv  + ... % SE: 1.5489E+02, PE:  11.42%, 95% CI: [-1.6655E+03,-1.0460E+03]
  +1.22951464614351E+03 .*        u_cv .* N2_cv  + ... % SE: 1.5515E+02, PE:  12.62%, 95% CI: [+9.1922E+02,+1.5398E+03]
  -8.24008685721299E+02 .*        w_cv .* N1_cv  + ... % SE: 1.5411E+02, PE:  18.70%, 95% CI: [-1.1322E+03,-5.1578E+02]
  +7.57503647803131E+02 .*        u_cv .* N5_cv  + ... % SE: 1.5603E+02, PE:  20.60%, 95% CI: [+4.4545E+02,+1.0696E+03]
  +4.52464365275576E+03 .*             N2_cv.^2  + ... % SE: 3.2514E+02, PE:   7.19%, 95% CI: [+3.8744E+03,+5.1749E+03]
  -3.96702974554432E+03 .*             N8_cv.^2  + ... % SE: 3.2747E+02, PE:   8.25%, 95% CI: [-4.6220E+03,-3.3121E+03]
  -4.09488784177950E+03 .*             N7_cv.^2  + ... % SE: 3.2856E+02, PE:   8.02%, 95% CI: [-4.7520E+03,-3.4378E+03]
  +4.05500429680544E+03 .*             N1_cv.^2  + ... % SE: 3.3327E+02, PE:   8.22%, 95% CI: [+3.3885E+03,+4.7215E+03]
  -2.14975222394767E+03 .*             N5_cv.^2  + ... % SE: 3.2608E+02, PE:  15.17%, 95% CI: [-2.8019E+03,-1.4976E+03]
  +1.65472678318789E+03 .*             N4_cv.^2 ;      % SE: 3.2518E+02, PE:  19.65%, 95% CI: [+1.0044E+03,+2.3051E+03]

% Mpitch Model
RSQ(5)=99.073; PSE(5)=8.2157E+05; PRESS(5)=3.7723E+08; NTERMS(5)=21;
Mpitch = ...
  +3.36608801583135E+03 .*                N3_cv  + ... % SE: 3.5360E+01, PE:   1.05%, 95% CI: [+3.2954E+03,+3.4368E+03]
  +3.36585097274724E+03 .*                N7_cv  + ... % SE: 3.5547E+01, PE:   1.06%, 95% CI: [+3.2948E+03,+3.4369E+03]
  +3.31040849145606E+03 .*                N1_cv  + ... % SE: 3.5387E+01, PE:   1.07%, 95% CI: [+3.2396E+03,+3.3812E+03]
  -3.46121268543939E+03 .*                N8_cv  + ... % SE: 3.5576E+01, PE:   1.03%, 95% CI: [-3.5324E+03,-3.3901E+03]
  -3.43256330611596E+03 .*                N2_cv  + ... % SE: 3.5585E+01, PE:   1.04%, 95% CI: [-3.5037E+03,-3.3614E+03]
  +3.33178868029127E+03 .*                N5_cv  + ... % SE: 3.5808E+01, PE:   1.07%, 95% CI: [+3.2602E+03,+3.4034E+03]
  -3.01125843449801E+03 .*                N6_cv  + ... % SE: 3.5540E+01, PE:   1.18%, 95% CI: [-3.0823E+03,-2.9402E+03]
  -3.02583911897422E+03 .*                N4_cv  + ... % SE: 3.5723E+01, PE:   1.18%, 95% CI: [-3.0973E+03,-2.9544E+03]
  +1.52211671705058E+03 .*                    1  + ... % SE: 5.5420E+01, PE:   3.64%, 95% CI: [+1.4113E+03,+1.6330E+03]
  +1.26630028132758E+03 .*                 u_cv  + ... % SE: 3.5304E+01, PE:   2.79%, 95% CI: [+1.1957E+03,+1.3369E+03]
  -8.96023077867465E+02 .*             N2_cv.^2  + ... % SE: 8.8536E+01, PE:   9.88%, 95% CI: [-1.0731E+03,-7.1895E+02]
  +3.70276569414965E+02 .*         u_cv .* w_cv  + ... % SE: 4.0673E+01, PE:  10.98%, 95% CI: [+2.8893E+02,+4.5162E+02]
  -3.48655313893460E+02 .*        v_cv .* N5_cv  + ... % SE: 4.1327E+01, PE:  11.85%, 95% CI: [-4.3131E+02,-2.6600E+02]
  +3.24685905728078E+02 .*        v_cv .* N3_cv  + ... % SE: 4.0843E+01, PE:  12.58%, 95% CI: [+2.4300E+02,+4.0637E+02]
  -8.73372083881372E+02 .*             N8_cv.^2  + ... % SE: 8.9389E+01, PE:  10.23%, 95% CI: [-1.0522E+03,-6.9459E+02]
  +6.77244110161533E+02 .*             N7_cv.^2  + ... % SE: 8.9633E+01, PE:  13.24%, 95% CI: [+4.9798E+02,+8.5651E+02]
  -9.12737121996814E+02 .*             N6_cv.^2  + ... % SE: 8.9551E+01, PE:   9.81%, 95% CI: [-1.0918E+03,-7.3364E+02]
  +6.59766895751317E+02 .*             N3_cv.^2  + ... % SE: 8.9834E+01, PE:  13.62%, 95% CI: [+4.8010E+02,+8.3943E+02]
  +6.13112982010672E+02 .*             N1_cv.^2  + ... % SE: 9.0438E+01, PE:  14.75%, 95% CI: [+4.3224E+02,+7.9399E+02]
  -7.24263536033292E+02 .*             N4_cv.^2  + ... % SE: 8.8824E+01, PE:  12.26%, 95% CI: [-9.0191E+02,-5.4662E+02]
  +5.63697379102415E+02 .*             N5_cv.^2 ;      % SE: 8.8458E+01, PE:  15.69%, 95% CI: [+3.8678E+02,+7.4061E+02]

% Myaw Model
RSQ(6)=97.719; PSE(6)=3.6704E+05; PRESS(6)=2.0045E+08; NTERMS(6)=46;
Myaw = ...
  +1.81593873787395E+03 .*                N5_cv  + ... % SE: 2.5151E+01, PE:   1.39%, 95% CI: [+1.7656E+03,+1.8662E+03]
  -1.80248538460866E+03 .*                N3_cv  + ... % SE: 2.4787E+01, PE:   1.38%, 95% CI: [-1.8521E+03,-1.7529E+03]
  +1.75999916896217E+03 .*                N4_cv  + ... % SE: 2.4951E+01, PE:   1.42%, 95% CI: [+1.7101E+03,+1.8099E+03]
  -1.74352069286684E+03 .*                N6_cv  + ... % SE: 2.4860E+01, PE:   1.43%, 95% CI: [-1.7932E+03,-1.6938E+03]
  +1.01674605342053E+03 .*                N8_cv  + ... % SE: 2.4890E+01, PE:   2.45%, 95% CI: [+9.6697E+02,+1.0665E+03]
  -9.71091061884177E+02 .*                N2_cv  + ... % SE: 2.4943E+01, PE:   2.57%, 95% CI: [-1.0210E+03,-9.2120E+02]
  +7.10973715093960E+02 .*                N1_cv  + ... % SE: 2.4747E+01, PE:   3.48%, 95% CI: [+6.6148E+02,+7.6047E+02]
  -6.66886964966401E+02 .*                N7_cv  + ... % SE: 2.4852E+01, PE:   3.73%, 95% CI: [-7.1659E+02,-6.1718E+02]
  -4.43268803400660E+02 .*        u_cv .* N1_cv  + ... % SE: 2.8613E+01, PE:   6.46%, 95% CI: [-5.0049E+02,-3.8604E+02]
  +4.76540389276124E+02 .*        u_cv .* N7_cv  + ... % SE: 2.8626E+01, PE:   6.01%, 95% CI: [+4.1929E+02,+5.3379E+02]
  -4.24623682722606E+02 .*        u_cv .* N2_cv  + ... % SE: 2.8706E+01, PE:   6.76%, 95% CI: [-4.8204E+02,-3.6721E+02]
  -4.04214784688248E+02 .*         u_cv .* v_cv  + ... % SE: 2.8462E+01, PE:   7.04%, 95% CI: [-4.6114E+02,-3.4729E+02]
  +3.83817515787226E+02 .*        u_cv .* N8_cv  + ... % SE: 2.8638E+01, PE:   7.46%, 95% CI: [+3.2654E+02,+4.4109E+02]
  +3.11922931216917E+02 .*              v_cv.^3  + ... % SE: 2.6830E+01, PE:   8.60%, 95% CI: [+2.5826E+02,+3.6558E+02]
  -2.63805197327525E+02 .*        v_cv .* N5_cv  + ... % SE: 2.8843E+01, PE:  10.93%, 95% CI: [-3.2149E+02,-2.0612E+02]
  +2.29837129218585E+02 .*        u_cv .* N6_cv  + ... % SE: 2.8815E+01, PE:  12.54%, 95% CI: [+1.7221E+02,+2.8747E+02]
  -2.26148488642935E+02 .*        u_cv .* N4_cv  + ... % SE: 2.8665E+01, PE:  12.68%, 95% CI: [-2.8348E+02,-1.6882E+02]
  +1.87572151783061E+02 .*        v_cv .* N6_cv  + ... % SE: 2.8793E+01, PE:  15.35%, 95% CI: [+1.2999E+02,+2.4516E+02]
  -1.61347623118904E+02 .*        u_cv .* N3_cv  + ... % SE: 2.8578E+01, PE:  17.71%, 95% CI: [-2.1850E+02,-1.0419E+02]
  +1.53685857095376E+02 .*        u_cv .* RA_cv  + ... % SE: 2.8602E+01, PE:  18.61%, 95% CI: [+9.6482E+01,+2.1089E+02]
  +1.58443013730393E+02 .*        v_cv .* N4_cv  + ... % SE: 2.8821E+01, PE:  18.19%, 95% CI: [+1.0080E+02,+2.1608E+02]
  -1.26629571877610E+02 .*        w_cv .* N1_cv  + ... % SE: 2.8505E+01, PE:  22.51%, 95% CI: [-1.8364E+02,-6.9620E+01]
  -1.40845846311922E+02 .*        v_cv .* N3_cv  + ... % SE: 2.8623E+01, PE:  20.32%, 95% CI: [-1.9809E+02,-8.3600E+01]
  +1.27210836997935E+02 .*            RUD_cv.^3  + ... % SE: 2.7094E+01, PE:  21.30%, 95% CI: [+7.3022E+01,+1.8140E+02]
  +1.41087821407665E+02 .*       u_cv .* RUD_cv  + ... % SE: 2.8698E+01, PE:  20.34%, 95% CI: [+8.3691E+01,+1.9848E+02]
  +1.59123245078173E+02 .*        w_cv .* N7_cv  + ... % SE: 2.8662E+01, PE:  18.01%, 95% CI: [+1.0180E+02,+2.1645E+02]
  +1.40260762660745E+02 .*         v_cv .* w_cv  + ... % SE: 2.8577E+01, PE:  20.37%, 95% CI: [+8.3108E+01,+1.9741E+02]
  +1.25319091707230E+02 .*        u_cv .* N5_cv  + ... % SE: 2.8859E+01, PE:  23.03%, 95% CI: [+6.7602E+01,+1.8304E+02]
  -5.85329082339292E+02 .*             N6_cv.^2  + ... % SE: 6.2973E+01, PE:  10.76%, 95% CI: [-7.1128E+02,-4.5938E+02]
  +5.02084249548667E+02 .*             N4_cv.^2  + ... % SE: 6.2172E+01, PE:  12.38%, 95% CI: [+3.7774E+02,+6.2643E+02]
  +4.75986299626889E+02 .*             N5_cv.^2  + ... % SE: 6.2053E+01, PE:  13.04%, 95% CI: [+3.5188E+02,+6.0009E+02]
  -4.96774638621817E+02 .*             N3_cv.^2  + ... % SE: 6.2858E+01, PE:  12.65%, 95% CI: [-6.2249E+02,-3.7106E+02]
  -4.17211710514684E+02 .*             N2_cv.^2  + ... % SE: 6.2035E+01, PE:  14.87%, 95% CI: [-5.4128E+02,-2.9314E+02]
  +3.06549338949819E+02 .*             N8_cv.^2  + ... % SE: 6.2448E+01, PE:  20.37%, 95% CI: [+1.8165E+02,+4.3145E+02]
  +3.28762297540609E+02 .*             N1_cv.^2  + ... % SE: 6.3235E+01, PE:  19.23%, 95% CI: [+2.0229E+02,+4.5523E+02]
  -3.13937205394082E+02 .*             N7_cv.^2  + ... % SE: 6.2774E+01, PE:  20.00%, 95% CI: [-4.3948E+02,-1.8839E+02]
  +1.18872046094021E+02 .*        v_cv .* N2_cv  + ... % SE: 2.8659E+01, PE:  24.11%, 95% CI: [+6.1554E+01,+1.7619E+02]
  -1.14647711129582E+02 .*        u_cv .* LA_cv  + ... % SE: 2.8491E+01, PE:  24.85%, 95% CI: [-1.7163E+02,-5.7665E+01]
  +1.04928199305839E+02 .*       N5_cv .* N7_cv  + ... % SE: 2.8982E+01, PE:  27.62%, 95% CI: [+4.6963E+01,+1.6289E+02]
  -1.13322393328670E+02 .*        w_cv .* N6_cv  + ... % SE: 2.8741E+01, PE:  25.36%, 95% CI: [-1.7080E+02,-5.5840E+01]
  -9.71749333580283E+01 .*       LE_cv .* N5_cv  + ... % SE: 2.9114E+01, PE:  29.96%, 95% CI: [-1.5540E+02,-3.8946E+01]
  -9.35026589881795E+01 .*       RA_cv .* N7_cv  + ... % SE: 2.8692E+01, PE:  30.69%, 95% CI: [-1.5089E+02,-3.6119E+01]
  -9.77808645328659E+01 .*       RE_cv .* N1_cv  + ... % SE: 2.8991E+01, PE:  29.65%, 95% CI: [-1.5576E+02,-3.9799E+01]
  +8.72054502622269E+01 .*        v_cv .* RA_cv  + ... % SE: 2.8605E+01, PE:  32.80%, 95% CI: [+2.9995E+01,+1.4442E+02]
  +1.90828021051500E+02 .*             RA_cv.^2  + ... % SE: 6.3083E+01, PE:  33.06%, 95% CI: [+6.4661E+01,+3.1699E+02]
  -8.28760825737303E+01 .*        v_cv .* N7_cv ;      % SE: 2.8664E+01, PE:  34.59%, 95% CI: [-1.4020E+02,-2.5549E+01]

% Flift Model
RSQ(7)=95.991; PSE(7)=1.5491E+06; PRESS(7)=9.9412E+08; NTERMS(7)=25;
Flift = ...
  +5.80765746835705E+03 .*                 u_cv  + ... % SE: 2.0625E+02, PE:   3.55%, 95% CI: [+5.3952E+03,+6.2202E+03]
  +6.70424262895095E+03 .*                    1  + ... % SE: 8.7256E+01, PE:   1.30%, 95% CI: [+6.5297E+03,+6.8788E+03]
  -4.54527482333780E+03 .*              u_cv.^2  + ... % SE: 1.3950E+02, PE:   3.07%, 95% CI: [-4.8243E+03,-4.2663E+03]
  +7.50932054082378E+02 .*        u_cv .* N2_cv  + ... % SE: 6.6826E+01, PE:   8.90%, 95% CI: [+6.1728E+02,+8.8458E+02]
  +7.04037888707969E+02 .*        u_cv .* N4_cv  + ... % SE: 6.6688E+01, PE:   9.47%, 95% CI: [+5.7066E+02,+8.3741E+02]
  +7.38146952525752E+02 .*        u_cv .* N6_cv  + ... % SE: 6.6638E+01, PE:   9.03%, 95% CI: [+6.0487E+02,+8.7142E+02]
  +6.83627049290547E+02 .*        u_cv .* N8_cv  + ... % SE: 6.6718E+01, PE:   9.76%, 95% CI: [+5.5019E+02,+8.1706E+02]
  +6.20322965561757E+02 .*        u_cv .* N7_cv  + ... % SE: 6.6537E+01, PE:  10.73%, 95% CI: [+4.8725E+02,+7.5340E+02]
  +6.32096923608896E+02 .*        u_cv .* N1_cv  + ... % SE: 6.6402E+01, PE:  10.51%, 95% CI: [+4.9929E+02,+7.6490E+02]
  +5.73342791293780E+02 .*        u_cv .* N5_cv  + ... % SE: 6.7192E+01, PE:  11.72%, 95% CI: [+4.3896E+02,+7.0773E+02]
  +5.53660990139259E+02 .*        u_cv .* N3_cv  + ... % SE: 6.6475E+01, PE:  12.01%, 95% CI: [+4.2071E+02,+6.8661E+02]
  +4.29531106345242E+02 .*                N8_cv  + ... % SE: 5.7992E+01, PE:  13.50%, 95% CI: [+3.1355E+02,+5.4552E+02]
  +3.29078321045665E+02 .*                 w_cv  + ... % SE: 5.7359E+01, PE:  17.43%, 95% CI: [+2.1436E+02,+4.4380E+02]
  +3.28141904524200E+02 .*                N6_cv  + ... % SE: 5.7939E+01, PE:  17.66%, 95% CI: [+2.1226E+02,+4.4402E+02]
  +3.14420043153120E+02 .*                N4_cv  + ... % SE: 5.8120E+01, PE:  18.48%, 95% CI: [+1.9818E+02,+4.3066E+02]
  +2.77757938488596E+02 .*                N2_cv  + ... % SE: 5.8046E+01, PE:  20.90%, 95% CI: [+1.6167E+02,+3.9385E+02]
  +9.20360356572537E+02 .*              u_cv.^3  + ... % SE: 2.2304E+02, PE:  24.23%, 95% CI: [+4.7429E+02,+1.3664E+03]
  +2.72501136724326E+02 .*         u_cv .* w_cv  + ... % SE: 6.6210E+01, PE:  24.30%, 95% CI: [+1.4008E+02,+4.0492E+02]
  +1.93633170764639E+02 .*                N7_cv  + ... % SE: 5.7888E+01, PE:  29.90%, 95% CI: [+7.7858E+01,+3.0941E+02]
  -4.36619238352776E+02 .*             LA_cv.^2  + ... % SE: 1.3933E+02, PE:  31.91%, 95% CI: [-7.1529E+02,-1.5795E+02]
  +1.86355845373914E+02 .*                N1_cv  + ... % SE: 5.7659E+01, PE:  30.94%, 95% CI: [+7.1038E+01,+3.0167E+02]
  +1.93555865481869E+02 .*                N5_cv  + ... % SE: 5.8408E+01, PE:  30.18%, 95% CI: [+7.6741E+01,+3.1037E+02]
  +1.80787199831842E+02 .*                N3_cv  + ... % SE: 5.7741E+01, PE:  31.94%, 95% CI: [+6.5305E+01,+2.9627E+02]
  -3.62431661768685E+02 .*              v_cv.^2  + ... % SE: 1.3913E+02, PE:  38.39%, 95% CI: [-6.4070E+02,-8.4168E+01]
  +3.33499786232032E+02 .*             N5_cv.^2 ;      % SE: 1.3733E+02, PE:  41.18%, 95% CI: [+5.8831E+01,+6.0817E+02]

% Fdrag Model
RSQ(8)=96.752; PSE(8)=8.2213E+05; PRESS(8)=5.2253E+08; NTERMS(8)=18;
Fdrag = ...
  +8.58300720835816E+03 .*                 w_cv  + ... % SE: 1.5099E+02, PE:   1.76%, 95% CI: [+8.2810E+03,+8.8850E+03]
  -1.50054207351494E+03 .*         u_cv .* w_cv  + ... % SE: 4.8344E+01, PE:   3.22%, 95% CI: [-1.5972E+03,-1.4039E+03]
  -3.28815815086333E+03 .*              w_cv.^3  + ... % SE: 1.6332E+02, PE:   4.97%, 95% CI: [-3.6148E+03,-2.9615E+03]
  +5.48063175428793E+02 .*        w_cv .* N2_cv  + ... % SE: 4.8547E+01, PE:   8.86%, 95% CI: [+4.5097E+02,+6.4516E+02]
  +5.39402662125567E+02 .*        w_cv .* N1_cv  + ... % SE: 4.8534E+01, PE:   9.00%, 95% CI: [+4.4234E+02,+6.3647E+02]
  +5.71109678179389E+02 .*        w_cv .* N8_cv  + ... % SE: 4.8789E+01, PE:   8.54%, 95% CI: [+4.7353E+02,+6.6869E+02]
  +5.41039813892424E+02 .*        w_cv .* N7_cv  + ... % SE: 4.8521E+01, PE:   8.97%, 95% CI: [+4.4400E+02,+6.3808E+02]
  +5.65140435767245E+02 .*        w_cv .* N4_cv  + ... % SE: 4.8829E+01, PE:   8.64%, 95% CI: [+4.6748E+02,+6.6280E+02]
  +5.52094307547071E+02 .*        w_cv .* N6_cv  + ... % SE: 4.8537E+01, PE:   8.79%, 95% CI: [+4.5502E+02,+6.4917E+02]
  +3.20021009906336E+02 .*              u_cv.^2  + ... % SE: 8.3799E+01, PE:  26.19%, 95% CI: [+1.5242E+02,+4.8762E+02]
  +5.15079558461332E+02 .*        w_cv .* N3_cv  + ... % SE: 4.8345E+01, PE:   9.39%, 95% CI: [+4.1839E+02,+6.1177E+02]
  +4.89365181122266E+02 .*        w_cv .* N5_cv  + ... % SE: 4.8913E+01, PE:  10.00%, 95% CI: [+3.9154E+02,+5.8719E+02]
  +1.75195140321387E+02 .*        v_cv .* N3_cv  + ... % SE: 4.8654E+01, PE:  27.77%, 95% CI: [+7.7886E+01,+2.7250E+02]
  -1.46712382099252E+02 .*        v_cv .* N5_cv  + ... % SE: 4.9087E+01, PE:  33.46%, 95% CI: [-2.4489E+02,-4.8538E+01]
  -1.40091390464943E+02 .*        v_cv .* N6_cv  + ... % SE: 4.8930E+01, PE:  34.93%, 95% CI: [-2.3795E+02,-4.2232E+01]
  +2.18642762697853E+02 .*              w_cv.^2  + ... % SE: 8.3689E+01, PE:  38.28%, 95% CI: [+5.1265E+01,+3.8602E+02]
  +1.22885833708311E+02 .*       LA_cv .* N7_cv  + ... % SE: 4.8895E+01, PE:  39.79%, 95% CI: [+2.5096E+01,+2.2068E+02]
  +1.03071944089930E+02 .*                 u_cv ;      % SE: 4.2003E+01, PE:  40.75%, 95% CI: [+1.9065E+01,+1.8708E+02]

% Modeling Metrics
Metrics.RSQ=RSQ;
Metrics.PSE=PSE;
Metrics.PRESS=PRESS;
Metrics.NTERMS=NTERMS;

C = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];

return