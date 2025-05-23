function [C,Metrics]=LCGabCFMa9999__Cp3FI_SWR_cv(u,Beta,Alpha,LA,RA,LE,RE,RUD)
% LCGabCFMa9999__Cp3FI_SWR_cv - Aerodynamic model for the LpC Glider
%
% DESCRIPTION: 
%   This script contains the aerodynamic model for LpC Glider. The script was
%   automatically generated using "GenModelCV.m" on 13-Jan-2021 21:51:46
%
% INPUTS:
%   List of column vectors containing the model explanatory variables 
%       The variables are in the following order:
%       [u,Beta,Alpha,LA,RA,LE,RE,RUD]
%       Units: deg  kts 
%
% OUTPUTS:
%   C - Matrix containing the model response variables in each column 
%       The variables are in the following order:
%       [CFx,CFy,CFz,CMx,CMy,CMz,CL,CD]
%       Units: nd 
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

%   Modeling Algorithm: SWR, Model Complexity: Cubic+3FI 
%   Model SF: 1, SWR Alpha: 0.9999 
%   Final Fval: [  15.31  15.31  15.31  15.31  15.32  15.31  15.31  15.32  ] 
%
%   MSD Explanatory Variables for each Response Variable: 
%    CFx: u_cv,Beta_cv,Alpha_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv
%    CFy: u_cv,Beta_cv,Alpha_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv
%    CFz: u_cv,Beta_cv,Alpha_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv
%    CMx: u_cv,Beta_cv,Alpha_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv
%    CMy: u_cv,Beta_cv,Alpha_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv
%    CMz: u_cv,Beta_cv,Alpha_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv
%    CL: u_cv,Beta_cv,Alpha_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv
%    CD: u_cv,Beta_cv,Alpha_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv
%    
%   Files used to develop model:
%   
%
%   Explantory variable ranges used to develop model:
%             u: [     +50,    +130] kts,  Nord:  3
%          Beta: [      -6,      +6] deg,  Nord:  3
%         Alpha: [      +0,     +12] deg,  Nord:  3
%            LA: [     -30,     +30] deg,  Nord:  3
%            RA: [     -30,     +30] deg,  Nord:  3
%            LE: [     -30,     +30] deg,  Nord:  3
%            RE: [     -30,     +30] deg,  Nord:  3
%           RUD: [     -30,     +30] deg,  Nord:  3
%
%   This model was developed using Rapid Aero Modeling (RAM) software.
%   Programs from the System IDentification Programs for AirCraft, or
%   SIDPAC, MATLAB software toolbox were used to determine the model
%   structure and parameter estimates shown below.
%
%
%   Rapid Aero Modeling (RAM) References:
%   [1] Murphy, P. C., Simmons, B. M., Hatke, D. B., Busan, R. C., "Rapid
%       Aero Modeling for Urban Air Mobility Aircraft in Wind-Tunnel
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
u_cv = ( u-(9.00000000000000E+01) )/(4.00000000000000E+01);
Beta_cv = ( Beta-(2.90847834705232E-09) )/(6.00000003797397E+00);
Alpha_cv = ( Alpha-(6.00000003411377E+00) )/(6.00000003411377E+00);
LA_cv = ( LA-(0.00000000000000E+00) )/(3.00000000000000E+01);
RA_cv = ( RA-(0.00000000000000E+00) )/(3.00000000000000E+01);
LE_cv = ( LE-(0.00000000000000E+00) )/(3.00000000000000E+01);
RE_cv = ( RE-(0.00000000000000E+00) )/(3.00000000000000E+01);
RUD_cv = ( RUD-(0.00000000000000E+00) )/(3.00000000000000E+01);




%Response variable polynomials

% CFx Model
RSQ(1)=99.83; PSE(1)=0.00026102; PRESS(1)=0.0079381; NTERMS(1)=31;
CFx = ...
  -2.99303539196512E-03 .*                 u_cv  + ... % SE: 3.2492E-04, PE:  10.86%, 95% CI: [-3.6429E-03,-2.3432E-03]
  -1.05034159846079E-01 .*             Alpha_cv  + ... % SE: 6.7360E-04, PE:   0.64%, 95% CI: [-1.0638E-01,-1.0369E-01]
  -1.35899464411477E-02 .*                LA_cv  + ... % SE: 3.0774E-04, PE:   2.26%, 95% CI: [-1.4205E-02,-1.2974E-02]
  -1.38400535458521E-02 .*                RA_cv  + ... % SE: 2.9947E-04, PE:   2.16%, 95% CI: [-1.4439E-02,-1.3241E-02]
  -1.17340214560307E-03 .*                LE_cv  + ... % SE: 1.5984E-04, PE:  13.62%, 95% CI: [-1.4931E-03,-8.5371E-04]
  +2.63894633813228E-03 .*              u_cv.^2  + ... % SE: 3.0905E-04, PE:  11.71%, 95% CI: [+2.0208E-03,+3.2570E-03]
  +1.78287535814232E-03 .*     u_cv .* Alpha_cv  + ... % SE: 2.4806E-04, PE:  13.91%, 95% CI: [+1.2868E-03,+2.2790E-03]
  -3.30999239745634E-02 .*          Alpha_cv.^2  + ... % SE: 3.1383E-04, PE:   0.95%, 95% CI: [-3.3728E-02,-3.2472E-02]
  +1.18492747600183E-03 .*        u_cv .* LA_cv  + ... % SE: 2.4775E-04, PE:  20.91%, 95% CI: [+6.8942E-04,+1.6804E-03]
  -1.60180322535008E-02 .*    Alpha_cv .* LA_cv  + ... % SE: 1.9578E-04, PE:   1.22%, 95% CI: [-1.6410E-02,-1.5626E-02]
  +1.88622007687991E-02 .*             LA_cv.^2  + ... % SE: 3.2157E-04, PE:   1.70%, 95% CI: [+1.8219E-02,+1.9505E-02]
  +1.53333811746034E-03 .*        u_cv .* RA_cv  + ... % SE: 2.4714E-04, PE:  16.12%, 95% CI: [+1.0390E-03,+2.0276E-03]
  -1.59892649784304E-02 .*    Alpha_cv .* RA_cv  + ... % SE: 1.9551E-04, PE:   1.22%, 95% CI: [-1.6380E-02,-1.5598E-02]
  -1.01406946620959E-03 .*       LA_cv .* RA_cv  + ... % SE: 1.9244E-04, PE:  18.98%, 95% CI: [-1.3989E-03,-6.2919E-04]
  +1.84385596173696E-02 .*             RA_cv.^2  + ... % SE: 3.1921E-04, PE:   1.73%, 95% CI: [+1.7800E-02,+1.9077E-02]
  -1.07391481735162E-03 .*    Alpha_cv .* LE_cv  + ... % SE: 1.9370E-04, PE:  18.04%, 95% CI: [-1.4613E-03,-6.8651E-04]
  +7.35828268378200E-03 .*             LE_cv.^2  + ... % SE: 3.2228E-04, PE:   4.38%, 95% CI: [+6.7137E-03,+8.0028E-03]
  -1.34033880521674E-03 .*    Alpha_cv .* RE_cv  + ... % SE: 1.9465E-04, PE:  14.52%, 95% CI: [-1.7296E-03,-9.5105E-04]
  -8.21919816455022E-04 .*       LE_cv .* RE_cv  + ... % SE: 1.9333E-04, PE:  23.52%, 95% CI: [-1.2086E-03,-4.3526E-04]
  +7.21782449919634E-03 .*             RE_cv.^2  + ... % SE: 3.1657E-04, PE:   4.39%, 95% CI: [+6.5847E-03,+7.8510E-03]
  +1.05124835773031E-02 .*            RUD_cv.^2  + ... % SE: 3.1505E-04, PE:   3.00%, 95% CI: [+9.8824E-03,+1.1143E-02]
  +3.02236845449034E-03 .*  u_cv.^2 .* Alpha_cv  + ... % SE: 3.7604E-04, PE:  12.44%, 95% CI: [+2.2703E-03,+3.7744E-03]
  +1.69023855202161E-03 .* Beta_cv.^2 .* Alpha_cv  + ... % SE: 4.0365E-04, PE:  23.88%, 95% CI: [+8.8293E-04,+2.4975E-03]
  +3.58386298366315E-03 .*  u_cv .* Alpha_cv.^2  + ... % SE: 4.3777E-04, PE:  12.22%, 95% CI: [+2.7083E-03,+4.4594E-03]
  +6.66642517919010E-03 .*          Alpha_cv.^3  + ... % SE: 6.9453E-04, PE:  10.42%, 95% CI: [+5.2774E-03,+8.0555E-03]
  +1.35135054283697E-03 .* u_cv .* Alpha_cv .* LA_cv  + ... % SE: 2.9634E-04, PE:  21.93%, 95% CI: [+7.5867E-04,+1.9440E-03]
  +2.62753222367311E-03 .* Alpha_cv.^2 .* LA_cv  + ... % SE: 3.8668E-04, PE:  14.72%, 95% CI: [+1.8542E-03,+3.4009E-03]
  +1.99794130626891E-03 .* u_cv .* Alpha_cv .* RA_cv  + ... % SE: 2.9676E-04, PE:  14.85%, 95% CI: [+1.4044E-03,+2.5915E-03]
  +2.86025702273366E-03 .* Alpha_cv.^2 .* RA_cv  + ... % SE: 3.7880E-04, PE:  13.24%, 95% CI: [+2.1026E-03,+3.6179E-03]
  -1.36045392566333E-03 .*             RE_cv.^3  + ... % SE: 1.7013E-04, PE:  12.51%, 95% CI: [-1.7007E-03,-1.0202E-03]
  -1.85018230791048E-02 .*                    1 ;      % SE: 2.6879E-04, PE:   1.45%, 95% CI: [-1.9039E-02,-1.7964E-02]

% CFy Model
RSQ(2)=99.34; PSE(2)=0.00012069; PRESS(2)=0.015106; NTERMS(2)=25;
CFy = ...
  -4.08625156085381E-02 .*              Beta_cv  + ... % SE: 8.9602E-04, PE:   2.19%, 95% CI: [-4.2655E-02,-3.9070E-02]
  -1.46880310077259E-02 .*                LA_cv  + ... % SE: 8.6858E-04, PE:   5.91%, 95% CI: [-1.6425E-02,-1.2951E-02]
  +1.26731627974367E-02 .*                RA_cv  + ... % SE: 4.2640E-04, PE:   3.36%, 95% CI: [+1.1820E-02,+1.3526E-02]
  -7.36914832316596E-03 .*                LE_cv  + ... % SE: 4.2890E-04, PE:   5.82%, 95% CI: [-8.2270E-03,-6.5113E-03]
  +6.67359439343339E-03 .*                RE_cv  + ... % SE: 4.1971E-04, PE:   6.29%, 95% CI: [+5.8342E-03,+7.5130E-03]
  -6.11684243276568E-02 .*               RUD_cv  + ... % SE: 9.2477E-04, PE:   1.51%, 95% CI: [-6.3018E-02,-5.9319E-02]
  -1.96741172117733E-03 .*  Beta_cv .* Alpha_cv  + ... % SE: 2.6925E-04, PE:  13.69%, 95% CI: [-2.5059E-03,-1.4289E-03]
  +4.18056587764394E-03 .*       u_cv .* RUD_cv  + ... % SE: 3.4506E-04, PE:   8.25%, 95% CI: [+3.4904E-03,+4.8707E-03]
  +6.82102664896420E-03 .*   Alpha_cv .* RUD_cv  + ... % SE: 2.7386E-04, PE:   4.01%, 95% CI: [+6.2733E-03,+7.3687E-03]
  +1.51049465027216E-03 .*      LA_cv .* RUD_cv  + ... % SE: 2.7302E-04, PE:  18.07%, 95% CI: [+9.6446E-04,+2.0565E-03]
  +1.24199582652745E-03 .*      RA_cv .* RUD_cv  + ... % SE: 2.7040E-04, PE:  21.77%, 95% CI: [+7.0119E-04,+1.7828E-03]
  +4.22963297336820E-03 .*           Beta_cv.^3  + ... % SE: 9.5215E-04, PE:  22.51%, 95% CI: [+2.3253E-03,+6.1339E-03]
  -3.32228461437784E-03 .* u_cv .* Beta_cv .* Alpha_cv  + ... % SE: 4.1104E-04, PE:  12.37%, 95% CI: [-4.1444E-03,-2.5002E-03]
  -1.64123443748984E-03 .* u_cv .* Beta_cv .* LA_cv  + ... % SE: 4.1066E-04, PE:  25.02%, 95% CI: [-2.4626E-03,-8.1991E-04]
  +4.12542063202353E-03 .*             LA_cv.^3  + ... % SE: 9.3225E-04, PE:  22.60%, 95% CI: [+2.2609E-03,+5.9899E-03]
  -2.30508194923530E-03 .*  Beta_cv.^2 .* RA_cv  + ... % SE: 5.3385E-04, PE:  23.16%, 95% CI: [-3.3728E-03,-1.2374E-03]
  -5.50471568402744E-03 .* Beta_cv.^2 .* RUD_cv  + ... % SE: 5.4681E-04, PE:   9.93%, 95% CI: [-6.5983E-03,-4.4111E-03]
  +1.83254289157756E-03 .* Beta_cv .* LE_cv .* RUD_cv  + ... % SE: 3.1843E-04, PE:  17.38%, 95% CI: [+1.1957E-03,+2.4694E-03]
  +2.25978436348110E-03 .* u_cv .* RE_cv .* RUD_cv  + ... % SE: 4.1287E-04, PE:  18.27%, 95% CI: [+1.4340E-03,+3.0855E-03]
  -2.78488563488629E-03 .* Beta_cv .* RE_cv .* RUD_cv  + ... % SE: 3.1734E-04, PE:  11.40%, 95% CI: [-3.4196E-03,-2.1502E-03]
  +3.06690956148481E-03 .* LE_cv .* RE_cv .* RUD_cv  + ... % SE: 3.1622E-04, PE:  10.31%, 95% CI: [+2.4345E-03,+3.6993E-03]
  +7.65330060604669E-03 .*   LE_cv .* RUD_cv.^2  + ... % SE: 5.3800E-04, PE:   7.03%, 95% CI: [+6.5773E-03,+8.7293E-03]
  -6.01783557653731E-03 .*   RE_cv .* RUD_cv.^2  + ... % SE: 5.2960E-04, PE:   8.80%, 95% CI: [-7.0770E-03,-4.9586E-03]
  +6.26038818805868E-03 .*            RUD_cv.^3  + ... % SE: 9.7912E-04, PE:  15.64%, 95% CI: [+4.3021E-03,+8.2186E-03]
  +2.99943729228131E-04 .*                    1 ;      % SE: 1.6739E-04, PE:  55.81%, 95% CI: [-3.4828E-05,+6.3472E-04]

% CFz Model
RSQ(3)=99.937; PSE(3)=0.008828; PRESS(3)=0.10986; NTERMS(3)=28;
CFz = ...
  +8.75903289338157E-03 .*                 u_cv  + ... % SE: 1.2238E-03, PE:  13.97%, 95% CI: [+6.3115E-03,+1.1207E-02]
  +5.19104775557944E-01 .*             Alpha_cv  + ... % SE: 1.6324E-03, PE:   0.31%, 95% CI: [+5.1584E-01,+5.2237E-01]
  +3.41428070710576E-01 .*                LA_cv  + ... % SE: 2.4741E-03, PE:   0.72%, 95% CI: [+3.3648E-01,+3.4638E-01]
  +3.34067813343603E-01 .*                RA_cv  + ... % SE: 2.4591E-03, PE:   0.74%, 95% CI: [+3.2915E-01,+3.3899E-01]
  +6.60585643631820E-02 .*                LE_cv  + ... % SE: 2.4066E-03, PE:   3.64%, 95% CI: [+6.1245E-02,+7.0872E-02]
  +5.94069051263524E-02 .*                RE_cv  + ... % SE: 1.1961E-03, PE:   2.01%, 95% CI: [+5.7015E-02,+6.1799E-02]
  -2.04341847295995E-02 .*           Beta_cv.^2  + ... % SE: 1.2104E-03, PE:   5.92%, 95% CI: [-2.2855E-02,-1.8013E-02]
  -2.39990365640757E-02 .*          Alpha_cv.^2  + ... % SE: 1.2013E-03, PE:   5.01%, 95% CI: [-2.6402E-02,-2.1597E-02]
  -1.56422557449911E-02 .*    Alpha_cv .* LA_cv  + ... % SE: 7.3902E-04, PE:   4.72%, 95% CI: [-1.7120E-02,-1.4164E-02]
  -2.87822435119032E-02 .*             LA_cv.^2  + ... % SE: 1.2200E-03, PE:   4.24%, 95% CI: [-3.1222E-02,-2.6342E-02]
  -1.53414114737747E-02 .*    Alpha_cv .* RA_cv  + ... % SE: 7.3925E-04, PE:   4.82%, 95% CI: [-1.6820E-02,-1.3863E-02]
  -2.98459851363695E-02 .*             RA_cv.^2  + ... % SE: 1.2091E-03, PE:   4.05%, 95% CI: [-3.2264E-02,-2.7428E-02]
  -7.81629621636397E-03 .*             LE_cv.^2  + ... % SE: 1.2282E-03, PE:  15.71%, 95% CI: [-1.0273E-02,-5.3598E-03]
  +3.39609902841195E-03 .*     Beta_cv .* RE_cv  + ... % SE: 7.2620E-04, PE:  21.38%, 95% CI: [+1.9437E-03,+4.8485E-03]
  -6.69122984596816E-03 .*             RE_cv.^2  + ... % SE: 1.2200E-03, PE:  18.23%, 95% CI: [-9.1311E-03,-4.2513E-03]
  -4.00382014628166E-03 .*    Beta_cv .* RUD_cv  + ... % SE: 7.3725E-04, PE:  18.41%, 95% CI: [-5.4783E-03,-2.5293E-03]
  -3.22285886625798E-03 .*      LE_cv .* RUD_cv  + ... % SE: 7.3091E-04, PE:  22.68%, 95% CI: [-4.6847E-03,-1.7610E-03]
  -7.77953881732776E-03 .*            RUD_cv.^2  + ... % SE: 1.1963E-03, PE:  15.38%, 95% CI: [-1.0172E-02,-5.3869E-03]
  -9.89499588544326E-03 .* Beta_cv.^2 .* Alpha_cv  + ... % SE: 1.5461E-03, PE:  15.63%, 95% CI: [-1.2987E-02,-6.8027E-03]
  -7.05934841024584E-03 .*  u_cv .* Alpha_cv.^2  + ... % SE: 1.6500E-03, PE:  23.37%, 95% CI: [-1.0359E-02,-3.7594E-03]
  -6.14299406629927E-03 .*  Beta_cv.^2 .* LA_cv  + ... % SE: 1.4981E-03, PE:  24.39%, 95% CI: [-9.1392E-03,-3.1468E-03]
  -7.06871592300997E-03 .* Alpha_cv .* LA_cv.^2  + ... % SE: 1.5446E-03, PE:  21.85%, 95% CI: [-1.0158E-02,-3.9795E-03]
  -7.72147329731536E-02 .*             LA_cv.^3  + ... % SE: 2.5807E-03, PE:   3.34%, 95% CI: [-8.2376E-02,-7.2053E-02]
  -8.10364258377586E-03 .* Alpha_cv .* RA_cv.^2  + ... % SE: 1.5251E-03, PE:  18.82%, 95% CI: [-1.1154E-02,-5.0534E-03]
  -7.31613227958684E-02 .*             RA_cv.^3  + ... % SE: 2.6117E-03, PE:   3.57%, 95% CI: [-7.8385E-02,-6.7938E-02]
  -1.19682902560598E-02 .*             LE_cv.^3  + ... % SE: 2.5626E-03, PE:  21.41%, 95% CI: [-1.7093E-02,-6.8432E-03]
  -7.29743639398966E-03 .*    LE_cv.^2 .* RE_cv  + ... % SE: 1.4880E-03, PE:  20.39%, 95% CI: [-1.0273E-02,-4.3214E-03]
  +7.58451647206869E-01 .*                    1 ;      % SE: 9.7046E-04, PE:   0.13%, 95% CI: [+7.5651E-01,+7.6039E-01]

% CMx Model
RSQ(4)=99.95; PSE(4)=0.00068076; PRESS(4)=0.0091063; NTERMS(4)=20;
CMx = ...
  -7.84931321797797E-03 .*              Beta_cv  + ... % SE: 1.7467E-04, PE:   2.23%, 95% CI: [-8.1986E-03,-7.5000E-03]
  +1.85104314998292E-01 .*                LA_cv  + ... % SE: 6.9697E-04, PE:   0.38%, 95% CI: [+1.8371E-01,+1.8650E-01]
  -1.85075447061081E-01 .*                RA_cv  + ... % SE: 7.0788E-04, PE:   0.38%, 95% CI: [-1.8649E-01,-1.8366E-01]
  +2.81917186545197E-03 .*                LE_cv  + ... % SE: 3.3721E-04, PE:  11.96%, 95% CI: [+2.1448E-03,+3.4936E-03]
  -4.13670667814794E-03 .*                RE_cv  + ... % SE: 1.7415E-04, PE:   4.21%, 95% CI: [-4.4850E-03,-3.7884E-03]
  -1.26388098498793E-02 .*               RUD_cv  + ... % SE: 1.7544E-04, PE:   1.39%, 95% CI: [-1.2990E-02,-1.2288E-02]
  -1.14614295050386E-03 .*     Beta_cv .* LA_cv  + ... % SE: 2.1186E-04, PE:  18.48%, 95% CI: [-1.5699E-03,-7.2243E-04]
  -8.01472250734303E-03 .*    Alpha_cv .* LA_cv  + ... % SE: 2.1340E-04, PE:   2.66%, 95% CI: [-8.4415E-03,-7.5879E-03]
  -1.73920600817313E-02 .*             LA_cv.^2  + ... % SE: 3.2445E-04, PE:   1.87%, 95% CI: [-1.8041E-02,-1.6743E-02]
  -1.04364439316886E-03 .*     Beta_cv .* RA_cv  + ... % SE: 2.1135E-04, PE:  20.25%, 95% CI: [-1.4663E-03,-6.2094E-04]
  +7.92408003375179E-03 .*    Alpha_cv .* RA_cv  + ... % SE: 2.1268E-04, PE:   2.68%, 95% CI: [+7.4987E-03,+8.3494E-03]
  +1.70078867601859E-02 .*             RA_cv.^2  + ... % SE: 3.2094E-04, PE:   1.89%, 95% CI: [+1.6366E-02,+1.7650E-02]
  +1.62629678400782E-03 .*   Alpha_cv .* RUD_cv  + ... % SE: 2.1409E-04, PE:  13.16%, 95% CI: [+1.1981E-03,+2.0545E-03]
  +1.73137286499545E-03 .*     u_cv.^2 .* LA_cv  + ... % SE: 4.0892E-04, PE:  23.62%, 95% CI: [+9.1352E-04,+2.5492E-03]
  -6.44411469901747E-03 .* Alpha_cv .* LA_cv.^2  + ... % SE: 3.5973E-04, PE:   5.58%, 95% CI: [-7.1636E-03,-5.7247E-03]
  -4.43571984413357E-02 .*             LA_cv.^3  + ... % SE: 7.3299E-04, PE:   1.65%, 95% CI: [-4.5823E-02,-4.2891E-02]
  +6.45707930417737E-03 .* Alpha_cv .* RA_cv.^2  + ... % SE: 3.5775E-04, PE:   5.54%, 95% CI: [+5.7416E-03,+7.1726E-03]
  +4.37946366737215E-02 .*             RA_cv.^3  + ... % SE: 7.5140E-04, PE:   1.72%, 95% CI: [+4.2292E-02,+4.5297E-02]
  +1.70752263545282E-03 .*   LE_cv .* RUD_cv.^2  + ... % SE: 4.2395E-04, PE:  24.83%, 95% CI: [+8.5962E-04,+2.5554E-03]
  +1.07866241868060E-06 .*                    1 ;      % SE: 2.3938E-04, PE: 22191.96%, 95% CI: [-4.7767E-04,+4.7983E-04]

% CMy Model
RSQ(5)=99.562; PSE(5)=0.0041095; PRESS(5)=0.22543; NTERMS(5)=45;
CMy = ...
  +1.79777366075283E-02 .*             Alpha_cv  + ... % SE: 2.0895E-03, PE:  11.62%, 95% CI: [+1.3799E-02,+2.2157E-02]
  -1.02627340064684E-02 .*                LA_cv  + ... % SE: 1.6662E-03, PE:  16.24%, 95% CI: [-1.3595E-02,-6.9304E-03]
  -3.91113549648267E-03 .*                RA_cv  + ... % SE: 8.4273E-04, PE:  21.55%, 95% CI: [-5.5966E-03,-2.2257E-03]
  -2.56315862803226E-01 .*                LE_cv  + ... % SE: 3.4616E-03, PE:   1.35%, 95% CI: [-2.6324E-01,-2.4939E-01]
  -2.66851902993243E-01 .*                RE_cv  + ... % SE: 3.5150E-03, PE:   1.32%, 95% CI: [-2.7388E-01,-2.5982E-01]
  -6.54629757349691E-03 .*              u_cv.^2  + ... % SE: 1.6278E-03, PE:  24.87%, 95% CI: [-9.8019E-03,-3.2907E-03]
  +2.10829497614012E-02 .*           Beta_cv.^2  + ... % SE: 1.6978E-03, PE:   8.05%, 95% CI: [+1.7687E-02,+2.4479E-02]
  -9.34915737544017E-03 .*     u_cv .* Alpha_cv  + ... % SE: 1.3146E-03, PE:  14.06%, 95% CI: [-1.1978E-02,-6.7200E-03]
  -4.54550384890668E-02 .*          Alpha_cv.^2  + ... % SE: 1.6945E-03, PE:   3.73%, 95% CI: [-4.8844E-02,-4.2066E-02]
  -4.29566489940834E-03 .*     Beta_cv .* LA_cv  + ... % SE: 1.0312E-03, PE:  24.01%, 95% CI: [-6.3580E-03,-2.2333E-03]
  -1.37763169363564E-02 .*    Alpha_cv .* LA_cv  + ... % SE: 1.0402E-03, PE:   7.55%, 95% CI: [-1.5857E-02,-1.1696E-02]
  +1.19143913204792E-02 .*             LA_cv.^2  + ... % SE: 1.7180E-03, PE:  14.42%, 95% CI: [+8.4784E-03,+1.5350E-02]
  +5.66974759439705E-03 .*     Beta_cv .* RA_cv  + ... % SE: 1.0241E-03, PE:  18.06%, 95% CI: [+3.6216E-03,+7.7179E-03]
  -1.42706780487988E-02 .*    Alpha_cv .* RA_cv  + ... % SE: 1.0358E-03, PE:   7.26%, 95% CI: [-1.6342E-02,-1.2199E-02]
  +1.75365050516073E-02 .*             RA_cv.^2  + ... % SE: 1.6947E-03, PE:   9.66%, 95% CI: [+1.4147E-02,+2.0926E-02]
  +6.83195478803959E-03 .*        u_cv .* LE_cv  + ... % SE: 1.2953E-03, PE:  18.96%, 95% CI: [+4.2414E-03,+9.4225E-03]
  +1.08354238894996E-02 .*     Beta_cv .* LE_cv  + ... % SE: 1.0191E-03, PE:   9.41%, 95% CI: [+8.7973E-03,+1.2874E-02]
  +2.97397218215292E-02 .*             LE_cv.^2  + ... % SE: 1.7233E-03, PE:   5.79%, 95% CI: [+2.6293E-02,+3.3186E-02]
  +6.86464742244200E-03 .*        u_cv .* RE_cv  + ... % SE: 1.3003E-03, PE:  18.94%, 95% CI: [+4.2640E-03,+9.4653E-03]
  -1.09378006754213E-02 .*     Beta_cv .* RE_cv  + ... % SE: 1.0254E-03, PE:   9.37%, 95% CI: [-1.2989E-02,-8.8870E-03]
  +5.06185930492530E-03 .*       LA_cv .* RE_cv  + ... % SE: 1.0373E-03, PE:  20.49%, 95% CI: [+2.9873E-03,+7.1364E-03]
  +7.99183423076666E-03 .*       LE_cv .* RE_cv  + ... % SE: 1.0316E-03, PE:  12.91%, 95% CI: [+5.9286E-03,+1.0055E-02]
  +3.00772066780489E-02 .*             RE_cv.^2  + ... % SE: 1.7072E-03, PE:   5.68%, 95% CI: [+2.6663E-02,+3.3492E-02]
  +1.41408176827555E-02 .*    Beta_cv .* RUD_cv  + ... % SE: 1.0360E-03, PE:   7.33%, 95% CI: [+1.2069E-02,+1.6213E-02]
  -5.12404521725384E-03 .*      LA_cv .* RUD_cv  + ... % SE: 1.0374E-03, PE:  20.25%, 95% CI: [-7.1988E-03,-3.0492E-03]
  +1.32770783390165E-02 .*      LE_cv .* RUD_cv  + ... % SE: 1.0284E-03, PE:   7.75%, 95% CI: [+1.1220E-02,+1.5334E-02]
  -9.75144521345346E-03 .*      RE_cv .* RUD_cv  + ... % SE: 1.0411E-03, PE:  10.68%, 95% CI: [-1.1834E-02,-7.6692E-03]
  +3.07819834084972E-02 .*            RUD_cv.^2  + ... % SE: 1.6761E-03, PE:   5.45%, 95% CI: [+2.7430E-02,+3.4134E-02]
  +9.69916485287343E-03 .*   u_cv .* Beta_cv.^2  + ... % SE: 1.3497E-03, PE:  13.92%, 95% CI: [+6.9997E-03,+1.2399E-02]
  +1.47079403975774E-02 .* Beta_cv.^2 .* Alpha_cv  + ... % SE: 2.1342E-03, PE:  14.51%, 95% CI: [+1.0440E-02,+1.8976E-02]
  +1.01895862427737E-02 .* Alpha_cv .* RA_cv.^2  + ... % SE: 2.1280E-03, PE:  20.88%, 95% CI: [+5.9336E-03,+1.4446E-02]
  +9.60186423062801E-03 .*    LA_cv .* LE_cv.^2  + ... % SE: 2.0915E-03, PE:  21.78%, 95% CI: [+5.4190E-03,+1.3785E-02]
  +2.43370466199213E-02 .*             LE_cv.^3  + ... % SE: 3.6726E-03, PE:  15.09%, 95% CI: [+1.6992E-02,+3.1682E-02]
  +1.46427734746949E-02 .*    LE_cv.^2 .* RE_cv  + ... % SE: 2.1208E-03, PE:  14.48%, 95% CI: [+1.0401E-02,+1.8884E-02]
  +1.24482041192361E-02 .*    LE_cv .* RE_cv.^2  + ... % SE: 2.1180E-03, PE:  17.01%, 95% CI: [+8.2122E-03,+1.6684E-02]
  +3.54421854597137E-02 .*             RE_cv.^3  + ... % SE: 3.7074E-03, PE:  10.46%, 95% CI: [+2.8027E-02,+4.2857E-02]
  +7.80814645899303E-03 .* Beta_cv .* Alpha_cv .* RUD_cv  + ... % SE: 1.2245E-03, PE:  15.68%, 95% CI: [+5.3592E-03,+1.0257E-02]
  +8.11995577769022E-03 .* u_cv .* LE_cv .* RUD_cv  + ... % SE: 1.5606E-03, PE:  19.22%, 95% CI: [+4.9988E-03,+1.1241E-02]
  +6.56562724609729E-03 .* Beta_cv .* LE_cv .* RUD_cv  + ... % SE: 1.2151E-03, PE:  18.51%, 95% CI: [+4.1355E-03,+8.9958E-03]
  +5.48530144017695E-03 .* Alpha_cv .* LE_cv .* RUD_cv  + ... % SE: 1.2317E-03, PE:  22.46%, 95% CI: [+3.0218E-03,+7.9488E-03]
  +7.33749055491550E-03 .* RA_cv .* LE_cv .* RUD_cv  + ... % SE: 1.2166E-03, PE:  16.58%, 95% CI: [+4.9042E-03,+9.7708E-03]
  +2.61381518556382E-02 .*   LE_cv.^2 .* RUD_cv  + ... % SE: 1.7298E-03, PE:   6.62%, 95% CI: [+2.2679E-02,+2.9598E-02]
  +5.39378017462775E-03 .* Beta_cv .* RE_cv .* RUD_cv  + ... % SE: 1.2141E-03, PE:  22.51%, 95% CI: [+2.9656E-03,+7.8219E-03]
  -2.73558185757554E-02 .*   RE_cv.^2 .* RUD_cv  + ... % SE: 1.7590E-03, PE:   6.43%, 95% CI: [-3.0874E-02,-2.3838E-02]
  +5.87774579754553E-02 .*                    1 ;      % SE: 1.4314E-03, PE:   2.44%, 95% CI: [+5.5915E-02,+6.1640E-02]

% CMz Model
RSQ(6)=99.381; PSE(6)=5.5146E-05; PRESS(6)=0.0053975; NTERMS(6)=32;
CMz = ...
  -1.84442035542186E-03 .*              Beta_cv  + ... % SE: 1.3277E-04, PE:   7.20%, 95% CI: [-2.1100E-03,-1.5789E-03]
  +7.24804306425458E-03 .*                LA_cv  + ... % SE: 1.3329E-04, PE:   1.84%, 95% CI: [+6.9815E-03,+7.5146E-03]
  -8.12460905637508E-03 .*                RA_cv  + ... % SE: 2.4904E-04, PE:   3.07%, 95% CI: [-8.6227E-03,-7.6265E-03]
  +4.46029498763726E-03 .*                LE_cv  + ... % SE: 2.5535E-04, PE:   5.72%, 95% CI: [+3.9496E-03,+4.9710E-03]
  -4.47486712229484E-03 .*                RE_cv  + ... % SE: 2.5028E-04, PE:   5.59%, 95% CI: [-4.9754E-03,-3.9743E-03]
  +4.34294213690080E-02 .*               RUD_cv  + ... % SE: 5.6697E-04, PE:   1.31%, 95% CI: [+4.2295E-02,+4.4563E-02]
  -8.85240026667269E-04 .*      u_cv .* Beta_cv  + ... % SE: 2.0377E-04, PE:  23.02%, 95% CI: [-1.2928E-03,-4.7769E-04]
  +8.32917579983166E-04 .*  Beta_cv .* Alpha_cv  + ... % SE: 1.6033E-04, PE:  19.25%, 95% CI: [+5.1226E-04,+1.1536E-03]
  +6.47446797888770E-03 .*    Alpha_cv .* LA_cv  + ... % SE: 1.6302E-04, PE:   2.52%, 95% CI: [+6.1484E-03,+6.8005E-03]
  -1.24295369150091E-02 .*             LA_cv.^2  + ... % SE: 2.4750E-04, PE:   1.99%, 95% CI: [-1.2925E-02,-1.1935E-02]
  -6.45227872188038E-03 .*    Alpha_cv .* RA_cv  + ... % SE: 1.6193E-04, PE:   2.51%, 95% CI: [-6.7761E-03,-6.1284E-03]
  +1.25269789262215E-02 .*             RA_cv.^2  + ... % SE: 2.4372E-04, PE:   1.95%, 95% CI: [+1.2040E-02,+1.3014E-02]
  -9.24955619770052E-04 .*        u_cv .* LE_cv  + ... % SE: 2.0338E-04, PE:  21.99%, 95% CI: [-1.3317E-03,-5.1820E-04]
  -2.79352059023120E-03 .*       u_cv .* RUD_cv  + ... % SE: 2.0630E-04, PE:   7.39%, 95% CI: [-3.2061E-03,-2.3809E-03]
  -4.72169428043720E-03 .*   Alpha_cv .* RUD_cv  + ... % SE: 1.6313E-04, PE:   3.45%, 95% CI: [-5.0479E-03,-4.3954E-03]
  -1.03117056293374E-03 .*      LA_cv .* RUD_cv  + ... % SE: 1.6268E-04, PE:  15.78%, 95% CI: [-1.3565E-03,-7.0581E-04]
  +8.43428460011791E-04 .*      LE_cv .* RUD_cv  + ... % SE: 1.6222E-04, PE:  19.23%, 95% CI: [+5.1899E-04,+1.1679E-03]
  +8.11057484201119E-04 .*      RE_cv .* RUD_cv  + ... % SE: 1.6203E-04, PE:  19.98%, 95% CI: [+4.8700E-04,+1.1351E-03]
  +1.00549580411866E-03 .* Beta_cv .* Alpha_cv .* LA_cv  + ... % SE: 1.9062E-04, PE:  18.96%, 95% CI: [+6.2425E-04,+1.3867E-03]
  +7.98229907334259E-04 .* Beta_cv .* Alpha_cv .* RA_cv  + ... % SE: 1.9065E-04, PE:  23.88%, 95% CI: [+4.1692E-04,+1.1795E-03]
  +1.42970924717210E-03 .* Alpha_cv.^2 .* RA_cv  + ... % SE: 3.1537E-04, PE:  22.06%, 95% CI: [+7.9898E-04,+2.0604E-03]
  +3.99500442148907E-03 .* Beta_cv.^2 .* RUD_cv  + ... % SE: 3.2983E-04, PE:   8.26%, 95% CI: [+3.3353E-03,+4.6547E-03]
  -1.19455470431269E-03 .* Beta_cv .* LE_cv .* RUD_cv  + ... % SE: 1.8992E-04, PE:  15.90%, 95% CI: [-1.5744E-03,-8.1472E-04]
  -8.28337846883909E-04 .* Alpha_cv .* LE_cv .* RUD_cv  + ... % SE: 1.9172E-04, PE:  23.14%, 95% CI: [-1.2118E-03,-4.4490E-04]
  -1.72499754908933E-03 .*   LE_cv.^2 .* RUD_cv  + ... % SE: 3.3342E-04, PE:  19.33%, 95% CI: [-2.3918E-03,-1.0582E-03]
  -1.17113183285434E-03 .* u_cv .* RE_cv .* RUD_cv  + ... % SE: 2.4951E-04, PE:  21.31%, 95% CI: [-1.6702E-03,-6.7211E-04]
  +1.60234053434116E-03 .* Beta_cv .* RE_cv .* RUD_cv  + ... % SE: 1.8908E-04, PE:  11.80%, 95% CI: [+1.2242E-03,+1.9805E-03]
  -1.94157949910484E-03 .* LE_cv .* RE_cv .* RUD_cv  + ... % SE: 1.8901E-04, PE:   9.74%, 95% CI: [-2.3196E-03,-1.5636E-03]
  -4.73305956986519E-03 .*   LE_cv .* RUD_cv.^2  + ... % SE: 3.2034E-04, PE:   6.77%, 95% CI: [-5.3737E-03,-4.0924E-03]
  +4.22461700474037E-03 .*   RE_cv .* RUD_cv.^2  + ... % SE: 3.1629E-04, PE:   7.49%, 95% CI: [+3.5920E-03,+4.8572E-03]
  -4.06377969151080E-03 .*            RUD_cv.^3  + ... % SE: 5.8945E-04, PE:  14.50%, 95% CI: [-5.2427E-03,-2.8849E-03]
  -8.94467508179160E-05 .*                    1 ;      % SE: 1.8195E-04, PE: 203.42%, 95% CI: [-4.5335E-04,+2.7445E-04]

% CL Model
RSQ(7)=99.934; PSE(7)=0.008792; PRESS(7)=0.11438; NTERMS(7)=28;
CL = ...
  +9.06673550170221E-03 .*                 u_cv  + ... % SE: 1.2484E-03, PE:  13.77%, 95% CI: [+6.5700E-03,+1.1564E-02]
  +5.19706319119622E-01 .*             Alpha_cv  + ... % SE: 1.6652E-03, PE:   0.32%, 95% CI: [+5.1638E-01,+5.2304E-01]
  +3.41392357744454E-01 .*                LA_cv  + ... % SE: 2.5238E-03, PE:   0.74%, 95% CI: [+3.3634E-01,+3.4644E-01]
  +3.33807589814093E-01 .*                RA_cv  + ... % SE: 2.5085E-03, PE:   0.75%, 95% CI: [+3.2879E-01,+3.3882E-01]
  +6.57847839270675E-02 .*                LE_cv  + ... % SE: 2.4550E-03, PE:   3.73%, 95% CI: [+6.0875E-02,+7.0695E-02]
  +5.92196244198022E-02 .*                RE_cv  + ... % SE: 1.2201E-03, PE:   2.06%, 95% CI: [+5.6779E-02,+6.1660E-02]
  -2.03833341355563E-02 .*           Beta_cv.^2  + ... % SE: 1.2347E-03, PE:   6.06%, 95% CI: [-2.2853E-02,-1.7914E-02]
  -1.96454642068798E-02 .*          Alpha_cv.^2  + ... % SE: 1.2254E-03, PE:   6.24%, 95% CI: [-2.2096E-02,-1.7195E-02]
  -1.55459901085290E-02 .*    Alpha_cv .* LA_cv  + ... % SE: 7.5387E-04, PE:   4.85%, 95% CI: [-1.7054E-02,-1.4038E-02]
  -3.04523066659278E-02 .*             LA_cv.^2  + ... % SE: 1.2446E-03, PE:   4.09%, 95% CI: [-3.2941E-02,-2.7963E-02]
  -1.52645280925407E-02 .*    Alpha_cv .* RA_cv  + ... % SE: 7.5411E-04, PE:   4.94%, 95% CI: [-1.6773E-02,-1.3756E-02]
  -3.15391063951655E-02 .*             RA_cv.^2  + ... % SE: 1.2334E-03, PE:   3.91%, 95% CI: [-3.4006E-02,-2.9072E-02]
  -8.55384093211058E-03 .*             LE_cv.^2  + ... % SE: 1.2529E-03, PE:  14.65%, 95% CI: [-1.1060E-02,-6.0480E-03]
  +3.32084784576594E-03 .*     Beta_cv .* RE_cv  + ... % SE: 7.4080E-04, PE:  22.31%, 95% CI: [+1.8393E-03,+4.8024E-03]
  -7.42933530765601E-03 .*             RE_cv.^2  + ... % SE: 1.2445E-03, PE:  16.75%, 95% CI: [-9.9183E-03,-4.9404E-03]
  -3.96237036336829E-03 .*    Beta_cv .* RUD_cv  + ... % SE: 7.5207E-04, PE:  18.98%, 95% CI: [-5.4665E-03,-2.4582E-03]
  -3.17909752000845E-03 .*      LE_cv .* RUD_cv  + ... % SE: 7.4561E-04, PE:  23.45%, 95% CI: [-4.6703E-03,-1.6879E-03]
  -8.82526872063435E-03 .*            RUD_cv.^2  + ... % SE: 1.2203E-03, PE:  13.83%, 95% CI: [-1.1266E-02,-6.3846E-03]
  -1.00719809449184E-02 .* Beta_cv.^2 .* Alpha_cv  + ... % SE: 1.5772E-03, PE:  15.66%, 95% CI: [-1.3226E-02,-6.9175E-03]
  -7.69053585896355E-03 .*  u_cv .* Alpha_cv.^2  + ... % SE: 1.6831E-03, PE:  21.89%, 95% CI: [-1.1057E-02,-4.3242E-03]
  -6.34330600271400E-03 .*  Beta_cv.^2 .* LA_cv  + ... % SE: 1.5282E-03, PE:  24.09%, 95% CI: [-9.3997E-03,-3.2869E-03]
  -8.94025690877694E-03 .* Alpha_cv .* LA_cv.^2  + ... % SE: 1.5756E-03, PE:  17.62%, 95% CI: [-1.2092E-02,-5.7890E-03]
  -7.69850952050803E-02 .*             LA_cv.^3  + ... % SE: 2.6325E-03, PE:   3.42%, 95% CI: [-8.2250E-02,-7.1720E-02]
  -9.84319752887521E-03 .* Alpha_cv .* RA_cv.^2  + ... % SE: 1.5558E-03, PE:  15.81%, 95% CI: [-1.2955E-02,-6.7316E-03]
  -7.28331900804136E-02 .*             RA_cv.^3  + ... % SE: 2.6641E-03, PE:   3.66%, 95% CI: [-7.8161E-02,-6.7505E-02]
  -1.20013648875985E-02 .*             LE_cv.^3  + ... % SE: 2.6141E-03, PE:  21.78%, 95% CI: [-1.7230E-02,-6.7732E-03]
  -7.42333168429975E-03 .*    LE_cv.^2 .* RE_cv  + ... % SE: 1.5179E-03, PE:  20.45%, 95% CI: [-1.0459E-02,-4.3875E-03]
  +7.56120962827665E-01 .*                    1 ;      % SE: 9.8997E-04, PE:   0.13%, 95% CI: [+7.5414E-01,+7.5810E-01]

% CD Model
RSQ(8)=99.571; PSE(8)=7.9322E-05; PRESS(8)=0.0044569; NTERMS(8)=43;
CD = ...
  -2.13293339066407E-03 .*                 u_cv  + ... % SE: 2.4023E-04, PE:  11.26%, 95% CI: [-2.6134E-03,-1.6525E-03]
  +2.73217409805777E-02 .*             Alpha_cv  + ... % SE: 5.1402E-04, PE:   1.88%, 95% CI: [+2.6294E-02,+2.8350E-02]
  +1.94567298405887E-02 .*                LA_cv  + ... % SE: 4.6913E-04, PE:   2.41%, 95% CI: [+1.8518E-02,+2.0395E-02]
  +1.95134928682612E-02 .*                RA_cv  + ... % SE: 4.9259E-04, PE:   2.52%, 95% CI: [+1.8528E-02,+2.0499E-02]
  +4.36273858676047E-03 .*                LE_cv  + ... % SE: 1.1843E-04, PE:   2.71%, 95% CI: [+4.1259E-03,+4.5996E-03]
  +4.34363343908167E-03 .*                RE_cv  + ... % SE: 1.1856E-04, PE:   2.73%, 95% CI: [+4.1065E-03,+4.5807E-03]
  +2.10876640504166E-03 .*              u_cv.^2  + ... % SE: 2.2855E-04, PE:  10.84%, 95% CI: [+1.6517E-03,+2.5659E-03]
  +1.22822187306043E-03 .*           Beta_cv.^2  + ... % SE: 2.3750E-04, PE:  19.34%, 95% CI: [+7.5323E-04,+1.7032E-03]
  +2.51213136021534E-03 .*     u_cv .* Alpha_cv  + ... % SE: 1.8400E-04, PE:   7.32%, 95% CI: [+2.1441E-03,+2.8801E-03]
  +1.73796253344126E-02 .*          Alpha_cv.^2  + ... % SE: 2.3563E-04, PE:   1.36%, 95% CI: [+1.6908E-02,+1.7851E-02]
  +1.05816363186368E-03 .*        u_cv .* LA_cv  + ... % SE: 1.8331E-04, PE:  17.32%, 95% CI: [+6.9155E-04,+1.4248E-03]
  +1.01544818825631E-02 .*    Alpha_cv .* LA_cv  + ... % SE: 1.4544E-04, PE:   1.43%, 95% CI: [+9.8636E-03,+1.0445E-02]
  +1.53077903943925E-02 .*             LA_cv.^2  + ... % SE: 2.3973E-04, PE:   1.57%, 95% CI: [+1.4828E-02,+1.5787E-02]
  +1.04464014064827E-03 .*        u_cv .* RA_cv  + ... % SE: 1.8311E-04, PE:  17.53%, 95% CI: [+6.7842E-04,+1.4109E-03]
  -6.39455174710301E-04 .*     Beta_cv .* RA_cv  + ... % SE: 1.4378E-04, PE:  22.49%, 95% CI: [-9.2702E-04,-3.5189E-04]
  +1.01183751954794E-02 .*    Alpha_cv .* RA_cv  + ... % SE: 1.4507E-04, PE:   1.43%, 95% CI: [+9.8282E-03,+1.0409E-02]
  -1.19577694125498E-03 .*       LA_cv .* RA_cv  + ... % SE: 1.4328E-04, PE:  11.98%, 95% CI: [-1.4823E-03,-9.0921E-04]
  +1.47467111636541E-02 .*             RA_cv.^2  + ... % SE: 2.3737E-04, PE:   1.61%, 95% CI: [+1.4272E-02,+1.5221E-02]
  -7.50146609640864E-04 .*     Beta_cv .* LE_cv  + ... % SE: 1.4276E-04, PE:  19.03%, 95% CI: [-1.0357E-03,-4.6463E-04]
  +4.54145367425137E-03 .*    Alpha_cv .* LE_cv  + ... % SE: 1.4349E-04, PE:   3.16%, 95% CI: [+4.2545E-03,+4.8284E-03]
  +6.45257413815954E-03 .*             LE_cv.^2  + ... % SE: 2.4068E-04, PE:   3.73%, 95% CI: [+5.9712E-03,+6.9339E-03]
  +7.64032742924384E-04 .*     Beta_cv .* RE_cv  + ... % SE: 1.4300E-04, PE:  18.72%, 95% CI: [+4.7803E-04,+1.0500E-03]
  +4.34594112603577E-03 .*    Alpha_cv .* RE_cv  + ... % SE: 1.4486E-04, PE:   3.33%, 95% CI: [+4.0562E-03,+4.6357E-03]
  -1.11256367066206E-03 .*       LE_cv .* RE_cv  + ... % SE: 1.4476E-04, PE:  13.01%, 95% CI: [-1.4021E-03,-8.2305E-04]
  +6.48803023648542E-03 .*             RE_cv.^2  + ... % SE: 2.3867E-04, PE:   3.68%, 95% CI: [+6.0107E-03,+6.9654E-03]
  +5.67969945630985E-03 .*    Beta_cv .* RUD_cv  + ... % SE: 1.4538E-04, PE:   2.56%, 95% CI: [+5.3889E-03,+5.9705E-03]
  -8.35708562483426E-04 .*      LE_cv .* RUD_cv  + ... % SE: 1.4383E-04, PE:  17.21%, 95% CI: [-1.1234E-03,-5.4805E-04]
  +8.99376766643751E-04 .*      RE_cv .* RUD_cv  + ... % SE: 1.4505E-04, PE:  16.13%, 95% CI: [+6.0928E-04,+1.1895E-03]
  +9.76991985942859E-03 .*            RUD_cv.^2  + ... % SE: 2.3427E-04, PE:   2.40%, 95% CI: [+9.3014E-03,+1.0238E-02]
  +2.15824726809682E-03 .*  u_cv.^2 .* Alpha_cv  + ... % SE: 2.7817E-04, PE:  12.89%, 95% CI: [+1.6019E-03,+2.7146E-03]
  -1.54344662940833E-03 .* Beta_cv.^2 .* Alpha_cv  + ... % SE: 3.0603E-04, PE:  19.83%, 95% CI: [-2.1555E-03,-9.3139E-04]
  +3.13469412870261E-03 .*  u_cv .* Alpha_cv.^2  + ... % SE: 3.2417E-04, PE:  10.34%, 95% CI: [+2.4864E-03,+3.7830E-03]
  +4.48404449665863E-03 .*          Alpha_cv.^3  + ... % SE: 5.2876E-04, PE:  11.79%, 95% CI: [+3.4265E-03,+5.5416E-03]
  +1.05443383169608E-03 .* u_cv .* Alpha_cv .* LA_cv  + ... % SE: 2.1986E-04, PE:  20.85%, 95% CI: [+6.1472E-04,+1.4941E-03]
  -3.38491308279465E-03 .* Alpha_cv .* LA_cv.^2  + ... % SE: 3.0769E-04, PE:   9.09%, 95% CI: [-4.0003E-03,-2.7695E-03]
  -4.96587029274066E-03 .*             LA_cv.^3  + ... % SE: 5.0336E-04, PE:  10.14%, 95% CI: [-5.9726E-03,-3.9592E-03]
  +1.26759614965392E-03 .*     u_cv.^2 .* RA_cv  + ... % SE: 2.7680E-04, PE:  21.84%, 95% CI: [+7.1400E-04,+1.8212E-03]
  +1.44763243985128E-03 .* u_cv .* Alpha_cv .* RA_cv  + ... % SE: 2.2078E-04, PE:  15.25%, 95% CI: [+1.0061E-03,+1.8892E-03]
  -4.10715035392033E-03 .* Alpha_cv .* RA_cv.^2  + ... % SE: 3.0372E-04, PE:   7.40%, 95% CI: [-4.7146E-03,-3.4997E-03]
  -5.54194305728809E-03 .*             RA_cv.^3  + ... % SE: 5.1359E-04, PE:   9.27%, 95% CI: [-6.5691E-03,-4.5148E-03]
  -7.96762406085324E-04 .* Beta_cv .* Alpha_cv .* RUD_cv  + ... % SE: 1.7241E-04, PE:  21.64%, 95% CI: [-1.1416E-03,-4.5194E-04]
  +7.99892546483687E-04 .* Beta_cv .* LE_cv .* RUD_cv  + ... % SE: 1.7080E-04, PE:  21.35%, 95% CI: [+4.5829E-04,+1.1415E-03]
  +6.15230545172541E-02 .*                    1 ;      % SE: 2.0048E-04, PE:   0.33%, 95% CI: [+6.1122E-02,+6.1924E-02]

% Modeling Metrics
Metrics.RSQ=RSQ;
Metrics.PSE=PSE;
Metrics.PRESS=PRESS;
Metrics.NTERMS=NTERMS;

C = [CFx,CFy,CFz,CMx,CMy,CMz,CL,CD];

return