function [C,Metrics]=LpC_Trans_R37t45_PCp2FI_MOF_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8,N9)
% LpC_Trans_R37t45_PCp2FI_MOF_cv - Aerodynamic model for the LpC Trans
%
% DESCRIPTION: 
%   This script contains the aerodynamic model for LpC Trans. The script was
%   automatically generated using "GenModelCV.m" on 17-Mar-2021 22:17:01
%
% INPUTS:
%   List of column vectors containing the model explanatory variables 
%       The variables are in the following order:
%       [u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8,N9]
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
%    Faxial: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv,N9_cv
%    Fside: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv,N9_cv
%    Fnormal: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv,N9_cv
%    Mroll: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv,N9_cv
%    Mpitch: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv,N9_cv
%    Myaw: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv,N9_cv
%    Flift: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv,N9_cv
%    Fdrag: u_cv,v_cv,w_cv,LA_cv,RA_cv,LE_cv,RE_cv,RUD_cv,N1_cv,N2_cv,N3_cv,N4_cv,N5_cv,N6_cv,N7_cv,N8_cv,N9_cv
%    
%   Files used to develop model:
%    BlockL4_R8_S1_B1_D37_output.dat  BlockL4_R8_S1_B2_D38_output.dat  BlockL4_R8_S1_B3_D39_output.dat  BlockL4_R8_S1_B4_D40_output.dat  BlockL4_R9_S2_B1_D42_output.dat  BlockL4_R9_S2_B2_D43_output.dat  BlockL4_R9_S2_B3_D44_output.dat  BlockL4_R9_S2_B4_D45_output.dat 
%
%   Explantory variable ranges used to develop model:
%             u: [   +1.25,  +13.75] kts,  Nord:  3
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
%            N9: [    +750,   +1750] rpm,  Nord:  3
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
u_cv = ( u-(7.50000000000000E+00) )/(6.25000000000000E+00);
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
N9_cv = ( N9-(1.25000000000000E+03) )/(5.00000000000000E+02);




%Response variable polynomials

% Faxial Model
RSQ(1)=98.673; PSE(1)=26020; PRESS(1)=1.7776E+07; NTERMS(1)=20;
Faxial = ...
  -2.12717802862534E+03 .*                    1  + ... % SE: 1.3813E+01, PE:   0.65%, 95% CI: [-2.1548E+03,-2.0996E+03]
  -1.61712734124326E+03 .*                N9_cv  + ... % SE: 6.5037E+00, PE:   0.40%, 95% CI: [-1.6301E+03,-1.6041E+03]
  +1.73755027949958E+02 .*              u_cv.^3  + ... % SE: 9.9255E+00, PE:   5.71%, 95% CI: [+1.5390E+02,+1.9361E+02]
  -1.07242633701379E+02 .*         v_cv .* w_cv  + ... % SE: 7.5143E+00, PE:   7.01%, 95% CI: [-1.2227E+02,-9.2214E+01]
  -6.78201900589275E+01 .*                 w_cv  + ... % SE: 6.5643E+00, PE:   9.68%, 95% CI: [-8.0949E+01,-5.4692E+01]
  -2.25507028421693E+02 .*             N9_cv.^2  + ... % SE: 1.4815E+01, PE:   6.57%, 95% CI: [-2.5514E+02,-1.9588E+02]
  -1.60451236437156E+02 .*              u_cv.^2  + ... % SE: 1.6480E+01, PE:  10.27%, 95% CI: [-1.9341E+02,-1.2749E+02]
  +1.44531368307254E+02 .*         u_cv .* v_cv  + ... % SE: 1.8983E+01, PE:  13.13%, 95% CI: [+1.0657E+02,+1.8250E+02]
  +5.12649034437857E+01 .*                RE_cv  + ... % SE: 6.5079E+00, PE:  12.69%, 95% CI: [+3.8249E+01,+6.4281E+01]
  +4.54314071724072E+01 .*             N4_cv.^3  + ... % SE: 7.0668E+00, PE:  15.55%, 95% CI: [+3.1298E+01,+5.9565E+01]
  -4.87489481203987E+01 .*        w_cv .* N9_cv  + ... % SE: 7.5696E+00, PE:  15.53%, 95% CI: [-6.3888E+01,-3.3610E+01]
  +4.49370330309972E+01 .*       RE_cv .* N9_cv  + ... % SE: 7.4797E+00, PE:  16.64%, 95% CI: [+2.9978E+01,+5.9897E+01]
  +3.79937393116470E+01 .*               RUD_cv  + ... % SE: 6.5378E+00, PE:  17.21%, 95% CI: [+2.4918E+01,+5.1069E+01]
  -3.18979581032707E+01 .*                LE_cv  + ... % SE: 6.5417E+00, PE:  20.51%, 95% CI: [-4.4981E+01,-1.8814E+01]
  -3.56909454094255E+01 .*        v_cv .* LE_cv  + ... % SE: 7.4768E+00, PE:  20.95%, 95% CI: [-5.0645E+01,-2.0737E+01]
  +6.80452664320351E+01 .*              v_cv.^2  + ... % SE: 1.5198E+01, PE:  22.33%, 95% CI: [+3.7650E+01,+9.8441E+01]
  -3.20354410325467E+01 .*              v_cv.^3  + ... % SE: 6.9597E+00, PE:  21.72%, 95% CI: [-4.5955E+01,-1.8116E+01]
  +2.86966161462083E+01 .*                N6_cv  + ... % SE: 6.5312E+00, PE:  22.76%, 95% CI: [+1.5634E+01,+4.1759E+01]
  +2.96860909590734E+01 .*      RUD_cv .* N9_cv  + ... % SE: 7.5603E+00, PE:  25.47%, 95% CI: [+1.4565E+01,+4.4807E+01]
  -2.86609858031528E+01 .*        v_cv .* N9_cv ;      % SE: 7.4513E+00, PE:  26.00%, 95% CI: [-4.3564E+01,-1.3758E+01]

% Fside Model
RSQ(2)=97.23; PSE(2)=6084.3; PRESS(2)=4.6956E+06; NTERMS(2)=30;
Fside = ...
  -4.37396267569628E+02 .*                 v_cv  + ... % SE: 3.2664E+00, PE:   0.75%, 95% CI: [-4.4393E+02,-4.3086E+02]
  +1.79191452244468E+02 .*                N5_cv  + ... % SE: 3.2946E+00, PE:   1.84%, 95% CI: [+1.7260E+02,+1.8578E+02]
  -1.43508487950605E+02 .*                N3_cv  + ... % SE: 1.1820E+01, PE:   8.24%, 95% CI: [-1.6715E+02,-1.1987E+02]
  +1.42675298561846E+02 .*                N6_cv  + ... % SE: 3.3032E+00, PE:   2.32%, 95% CI: [+1.3607E+02,+1.4928E+02]
  -1.32217827524951E+02 .*                N4_cv  + ... % SE: 3.3158E+00, PE:   2.51%, 95% CI: [-1.3885E+02,-1.2559E+02]
  -1.22175755548089E+02 .*         v_cv .* w_cv  + ... % SE: 3.8022E+00, PE:   3.11%, 95% CI: [-1.2978E+02,-1.1457E+02]
  -7.83271343899356E+01 .*        v_cv .* N9_cv  + ... % SE: 3.7755E+00, PE:   4.82%, 95% CI: [-8.5878E+01,-7.0776E+01]
  -6.30958204509640E+01 .*               RUD_cv  + ... % SE: 3.3098E+00, PE:   5.25%, 95% CI: [-6.9715E+01,-5.6476E+01]
  -4.53084417164416E+01 .*      RUD_cv .* N9_cv  + ... % SE: 3.8300E+00, PE:   8.45%, 95% CI: [-5.2968E+01,-3.7648E+01]
  -3.98808189490408E+01 .*        v_cv .* N8_cv  + ... % SE: 3.7950E+00, PE:   9.52%, 95% CI: [-4.7471E+01,-3.2291E+01]
  -3.59927442945827E+01 .*        v_cv .* N2_cv  + ... % SE: 3.7934E+00, PE:  10.54%, 95% CI: [-4.3580E+01,-2.8406E+01]
  +3.96448335966495E+01 .*             N5_cv.^2  + ... % SE: 7.2636E+00, PE:  18.32%, 95% CI: [+2.5118E+01,+5.4172E+01]
  +7.09940605345483E+01 .*         u_cv .* v_cv  + ... % SE: 9.5748E+00, PE:  13.49%, 95% CI: [+5.1844E+01,+9.0144E+01]
  -2.74735007838393E+01 .*        v_cv .* N5_cv  + ... % SE: 3.7870E+00, PE:  13.78%, 95% CI: [-3.5047E+01,-1.9900E+01]
  -2.23515165673529E+01 .*        v_cv .* N3_cv  + ... % SE: 3.7878E+00, PE:  16.95%, 95% CI: [-2.9927E+01,-1.4776E+01]
  -1.98429804902305E+01 .*        v_cv .* N4_cv  + ... % SE: 3.8150E+00, PE:  19.23%, 95% CI: [-2.7473E+01,-1.2213E+01]
  -1.93335561726830E+01 .*        v_cv .* N1_cv  + ... % SE: 3.7862E+00, PE:  19.58%, 95% CI: [-2.6906E+01,-1.1761E+01]
  +1.70079226182366E+01 .*                N9_cv  + ... % SE: 3.2921E+00, PE:  19.36%, 95% CI: [+1.0424E+01,+2.3592E+01]
  -4.21065051468831E+01 .*             N3_cv.^2  + ... % SE: 7.2106E+00, PE:  17.12%, 95% CI: [-5.6528E+01,-2.7685E+01]
  +3.06676503863494E+01 .*             N6_cv.^2  + ... % SE: 7.2996E+00, PE:  23.80%, 95% CI: [+1.6069E+01,+4.5267E+01]
  -1.55982451951581E+01 .*        v_cv .* N7_cv  + ... % SE: 3.7870E+00, PE:  24.28%, 95% CI: [-2.3172E+01,-8.0243E+00]
  +1.52114096166321E+01 .*             N7_cv.^3  + ... % SE: 3.5570E+00, PE:  23.38%, 95% CI: [+8.0974E+00,+2.2325E+01]
  -1.33479685359622E+01 .*        v_cv .* N6_cv  + ... % SE: 3.7864E+00, PE:  28.37%, 95% CI: [-2.0921E+01,-5.7752E+00]
  -1.13748013964412E+01 .*             N1_cv.^3  + ... % SE: 3.5708E+00, PE:  31.39%, 95% CI: [-1.8516E+01,-4.2331E+00]
  -1.12731889352532E+01 .*       N2_cv .* N8_cv  + ... % SE: 3.8412E+00, PE:  34.07%, 95% CI: [-1.8956E+01,-3.5908E+00]
  +1.12146706924462E+01 .*       N3_cv .* N9_cv  + ... % SE: 3.8217E+00, PE:  34.08%, 95% CI: [+3.5712E+00,+1.8858E+01]
  -3.47107724021044E+01 .*             N3_cv.^3  + ... % SE: 1.2773E+01, PE:  36.80%, 95% CI: [-6.0257E+01,-9.1646E+00]
  +1.05068340942212E+01 .*      RUD_cv .* N8_cv  + ... % SE: 3.8399E+00, PE:  36.55%, 95% CI: [+2.8269E+00,+1.8187E+01]
  +1.08525892868072E+01 .*       LE_cv .* N7_cv  + ... % SE: 3.8309E+00, PE:  35.30%, 95% CI: [+3.1907E+00,+1.8514E+01]
  +1.06237369233834E+01 .*      RUD_cv .* N6_cv ;      % SE: 3.8378E+00, PE:  36.12%, 95% CI: [+2.9482E+00,+1.8299E+01]

% Fnormal Model
RSQ(3)=98.761; PSE(3)=65982; PRESS(3)=4.519E+07; NTERMS(3)=23;
Fnormal = ...
  +7.73675511163100E+03 .*                    1  + ... % SE: 1.3829E+01, PE:   0.18%, 95% CI: [+7.7091E+03,+7.7644E+03]
  +9.87326625341699E+02 .*                N8_cv  + ... % SE: 1.0346E+01, PE:   1.05%, 95% CI: [+9.6663E+02,+1.0080E+03]
  +9.74048519818653E+02 .*                N4_cv  + ... % SE: 1.0374E+01, PE:   1.07%, 95% CI: [+9.5330E+02,+9.9480E+02]
  +9.67610326292873E+02 .*                N2_cv  + ... % SE: 1.0376E+01, PE:   1.07%, 95% CI: [+9.4686E+02,+9.8836E+02]
  +9.58685996905004E+02 .*                N6_cv  + ... % SE: 1.0352E+01, PE:   1.08%, 95% CI: [+9.3798E+02,+9.7939E+02]
  +8.32063865637987E+02 .*                N7_cv  + ... % SE: 1.0325E+01, PE:   1.24%, 95% CI: [+8.1141E+02,+8.5271E+02]
  +8.32707768274006E+02 .*                N1_cv  + ... % SE: 1.0335E+01, PE:   1.24%, 95% CI: [+8.1204E+02,+8.5338E+02]
  +7.77802288231800E+02 .*                N5_cv  + ... % SE: 1.0309E+01, PE:   1.33%, 95% CI: [+7.5718E+02,+7.9842E+02]
  +7.79646366150839E+02 .*                N3_cv  + ... % SE: 1.0347E+01, PE:   1.33%, 95% CI: [+7.5895E+02,+8.0034E+02]
  +2.46130891674938E+02 .*             N6_cv.^2  + ... % SE: 2.6078E+01, PE:  10.60%, 95% CI: [+1.9398E+02,+2.9829E+02]
  +4.44451403508929E+02 .*                 w_cv  + ... % SE: 1.0397E+01, PE:   2.34%, 95% CI: [+4.2366E+02,+4.6525E+02]
  +2.54321476590894E+02 .*             N7_cv.^2  + ... % SE: 2.6063E+01, PE:  10.25%, 95% CI: [+2.0220E+02,+3.0645E+02]
  +2.47126688845058E+02 .*             N4_cv.^2  + ... % SE: 2.6066E+01, PE:  10.55%, 95% CI: [+1.9500E+02,+2.9926E+02]
  +2.25912644693297E+02 .*             N8_cv.^2  + ... % SE: 2.5990E+01, PE:  11.50%, 95% CI: [+1.7393E+02,+2.7789E+02]
  +2.15313093290827E+02 .*             N1_cv.^2  + ... % SE: 2.6097E+01, PE:  12.12%, 95% CI: [+1.6312E+02,+2.6751E+02]
  +2.00220894045024E+02 .*             N3_cv.^2  + ... % SE: 2.5902E+01, PE:  12.94%, 95% CI: [+1.4842E+02,+2.5202E+02]
  +1.97348444718133E+02 .*             N2_cv.^2  + ... % SE: 2.6179E+01, PE:  13.27%, 95% CI: [+1.4499E+02,+2.4971E+02]
  -8.81029262330872E+01 .*        v_cv .* N5_cv  + ... % SE: 1.1829E+01, PE:  13.43%, 95% CI: [-1.1176E+02,-6.4445E+01]
  -6.68045622329369E+01 .*                N9_cv  + ... % SE: 1.0300E+01, PE:  15.42%, 95% CI: [-8.7405E+01,-4.6205E+01]
  -7.60581450063005E+01 .*        w_cv .* N1_cv  + ... % SE: 1.2011E+01, PE:  15.79%, 95% CI: [-1.0008E+02,-5.2036E+01]
  +7.48657179392360E+01 .*        v_cv .* N3_cv  + ... % SE: 1.1851E+01, PE:  15.83%, 95% CI: [+5.1164E+01,+9.8568E+01]
  +1.67087760246563E+02 .*             N5_cv.^2  + ... % SE: 2.6020E+01, PE:  15.57%, 95% CI: [+1.1505E+02,+2.1913E+02]
  -1.18344900552038E+02 .*             RE_cv.^2 ;      % SE: 2.6458E+01, PE:  22.36%, 95% CI: [-1.7126E+02,-6.5429E+01]

% Mroll Model
RSQ(4)=99.32; PSE(4)=9.9934E+06; PRESS(4)=5.0775E+09; NTERMS(4)=25;
Mroll = ...
  +1.85664136386092E+04 .*                N2_cv  + ... % SE: 1.1026E+02, PE:   0.59%, 95% CI: [+1.8346E+04,+1.8787E+04]
  -1.84706399006133E+04 .*                N8_cv  + ... % SE: 1.0973E+02, PE:   0.59%, 95% CI: [-1.8690E+04,-1.8251E+04]
  +1.57431004913356E+04 .*                N1_cv  + ... % SE: 1.0962E+02, PE:   0.70%, 95% CI: [+1.5524E+04,+1.5962E+04]
  -1.56623120213558E+04 .*                N7_cv  + ... % SE: 1.0955E+02, PE:   0.70%, 95% CI: [-1.5881E+04,-1.5443E+04]
  +7.75557688637651E+03 .*                N4_cv  + ... % SE: 1.1000E+02, PE:   1.42%, 95% CI: [+7.5356E+03,+7.9756E+03]
  -7.43953788006052E+03 .*                N6_cv  + ... % SE: 1.0970E+02, PE:   1.47%, 95% CI: [-7.6589E+03,-7.2201E+03]
  +6.91815064148831E+03 .*                N3_cv  + ... % SE: 1.0973E+02, PE:   1.59%, 95% CI: [+6.6987E+03,+7.1376E+03]
  -6.74703851071514E+03 .*                N5_cv  + ... % SE: 1.0941E+02, PE:   1.62%, 95% CI: [-6.9659E+03,-6.5282E+03]
  -5.73430525801340E+03 .*                 v_cv  + ... % SE: 1.0842E+02, PE:   1.89%, 95% CI: [-5.9512E+03,-5.5175E+03]
  -5.22847147324677E+03 .*         v_cv .* w_cv  + ... % SE: 1.2623E+02, PE:   2.41%, 95% CI: [-5.4809E+03,-4.9760E+03]
  -2.78047936488761E+03 .*                    1  + ... % SE: 1.4616E+02, PE:   5.26%, 95% CI: [-3.0728E+03,-2.4882E+03]
  -2.37305042133636E+03 .*                N9_cv  + ... % SE: 1.0923E+02, PE:   4.60%, 95% CI: [-2.5915E+03,-2.1546E+03]
  -4.12171702537064E+03 .*             N8_cv.^2  + ... % SE: 2.7441E+02, PE:   6.66%, 95% CI: [-4.6705E+03,-3.5729E+03]
  +4.04200952961083E+03 .*             N1_cv.^2  + ... % SE: 2.7448E+02, PE:   6.79%, 95% CI: [+3.4930E+03,+4.5910E+03]
  -3.68028265999297E+03 .*             N7_cv.^2  + ... % SE: 2.7461E+02, PE:   7.46%, 95% CI: [-4.2295E+03,-3.1311E+03]
  +3.42704623700778E+03 .*             N2_cv.^2  + ... % SE: 2.7458E+02, PE:   8.01%, 95% CI: [+2.8779E+03,+3.9762E+03]
  +1.07096607251532E+03 .*        w_cv .* N7_cv  + ... % SE: 1.2757E+02, PE:  11.91%, 95% CI: [+8.1582E+02,+1.3261E+03]
  -9.71678244526055E+02 .*        w_cv .* N1_cv  + ... % SE: 1.2755E+02, PE:  13.13%, 95% CI: [-1.2268E+03,-7.1658E+02]
  +2.13835517178317E+03 .*             N4_cv.^2  + ... % SE: 2.7498E+02, PE:  12.86%, 95% CI: [+1.5884E+03,+2.6883E+03]
  -2.01570365474175E+03 .*             N6_cv.^2  + ... % SE: 2.7492E+02, PE:  13.64%, 95% CI: [-2.5655E+03,-1.4659E+03]
  -8.15666254262243E+02 .*        v_cv .* N4_cv  + ... % SE: 1.2641E+02, PE:  15.50%, 95% CI: [-1.0685E+03,-5.6285E+02]
  +2.05449153732571E+03 .*         u_cv .* v_cv  + ... % SE: 3.1760E+02, PE:  15.46%, 95% CI: [+1.4193E+03,+2.6897E+03]
  -1.74097020645388E+03 .*             N5_cv.^2  + ... % SE: 2.7436E+02, PE:  15.76%, 95% CI: [-2.2897E+03,-1.1922E+03]
  +1.70672890215531E+03 .*             N3_cv.^2  + ... % SE: 2.7310E+02, PE:  16.00%, 95% CI: [+1.1605E+03,+2.2529E+03]
  -6.91719714720120E+02 .*        v_cv .* N6_cv ;      % SE: 1.2577E+02, PE:  18.18%, 95% CI: [-9.4325E+02,-4.4019E+02]

% Mpitch Model
RSQ(5)=98.408; PSE(5)=1.159E+06; PRESS(5)=7.3886E+08; NTERMS(5)=37;
Mpitch = ...
  -3.34414998789482E+03 .*                N8_cv  + ... % SE: 4.1380E+01, PE:   1.24%, 95% CI: [-3.4269E+03,-3.2614E+03]
  +3.27993805025683E+03 .*                N7_cv  + ... % SE: 4.1254E+01, PE:   1.26%, 95% CI: [+3.1974E+03,+3.3624E+03]
  -3.33893627116021E+03 .*                N2_cv  + ... % SE: 4.1487E+01, PE:   1.24%, 95% CI: [-3.4219E+03,-3.2560E+03]
  +3.24727396832216E+03 .*                N3_cv  + ... % SE: 4.1326E+01, PE:   1.27%, 95% CI: [+3.1646E+03,+3.3299E+03]
  +3.21240839828540E+03 .*                N1_cv  + ... % SE: 4.1304E+01, PE:   1.29%, 95% CI: [+3.1298E+03,+3.2950E+03]
  +3.20637224000411E+03 .*                N5_cv  + ... % SE: 4.1246E+01, PE:   1.29%, 95% CI: [+3.1239E+03,+3.2889E+03]
  -3.00666385134919E+03 .*                N4_cv  + ... % SE: 4.1392E+01, PE:   1.38%, 95% CI: [-3.0894E+03,-2.9239E+03]
  -2.96639016999605E+03 .*                N6_cv  + ... % SE: 4.1364E+01, PE:   1.39%, 95% CI: [-3.0491E+03,-2.8837E+03]
  -1.15294708878692E+03 .*                    1  + ... % SE: 5.5151E+01, PE:   4.78%, 95% CI: [-1.2632E+03,-1.0426E+03]
  -1.97342592088026E+03 .*                N9_cv  + ... % SE: 4.1149E+01, PE:   2.09%, 95% CI: [-2.0557E+03,-1.8911E+03]
  +1.51381926828392E+03 .*                 u_cv  + ... % SE: 5.4278E+01, PE:   3.59%, 95% CI: [+1.4053E+03,+1.6224E+03]
  -1.00523321978348E+03 .*             N4_cv.^2  + ... % SE: 1.0411E+02, PE:  10.36%, 95% CI: [-1.2135E+03,-7.9701E+02]
  -5.14453434656685E+02 .*        v_cv .* N5_cv  + ... % SE: 4.7278E+01, PE:   9.19%, 95% CI: [-6.0901E+02,-4.1990E+02]
  +4.78435977855042E+02 .*        v_cv .* N3_cv  + ... % SE: 4.7432E+01, PE:   9.91%, 95% CI: [+3.8357E+02,+5.7330E+02]
  -5.06156219354099E+02 .*              w_cv.^3  + ... % SE: 4.5025E+01, PE:   8.90%, 95% CI: [-5.9621E+02,-4.1611E+02]
  +1.23723518080419E+03 .*         u_cv .* w_cv  + ... % SE: 1.2191E+02, PE:   9.85%, 95% CI: [+9.9341E+02,+1.4811E+03]
  -4.47572736298936E+02 .*                LE_cv  + ... % SE: 4.1314E+01, PE:   9.23%, 95% CI: [-5.3020E+02,-3.6494E+02]
  -2.56377007851201E+02 .*                RE_cv  + ... % SE: 4.1045E+01, PE:  16.01%, 95% CI: [-3.3847E+02,-1.7429E+02]
  +6.60462310983088E+02 .*              w_cv.^2  + ... % SE: 1.0240E+02, PE:  15.50%, 95% CI: [+4.5567E+02,+8.6526E+02]
  -9.89909135606455E+02 .*             N8_cv.^2  + ... % SE: 1.0411E+02, PE:  10.52%, 95% CI: [-1.1981E+03,-7.8170E+02]
  -9.87538476854730E+02 .*             N6_cv.^2  + ... % SE: 1.0446E+02, PE:  10.58%, 95% CI: [-1.1965E+03,-7.7862E+02]
  +6.54110843193308E+02 .*             N3_cv.^2  + ... % SE: 1.0373E+02, PE:  15.86%, 95% CI: [+4.4666E+02,+8.6156E+02]
  +6.61805896967710E+02 .*             N1_cv.^2  + ... % SE: 1.0434E+02, PE:  15.77%, 95% CI: [+4.5312E+02,+8.7049E+02]
  -7.30906113012418E+02 .*             N2_cv.^2  + ... % SE: 1.0484E+02, PE:  14.34%, 95% CI: [-9.4059E+02,-5.2122E+02]
  +5.95858544189722E+02 .*             N5_cv.^2  + ... % SE: 1.0417E+02, PE:  17.48%, 95% CI: [+3.8752E+02,+8.0420E+02]
  -2.89668114567073E+02 .*         v_cv .* w_cv  + ... % SE: 4.7651E+01, PE:  16.45%, 95% CI: [-3.8497E+02,-1.9437E+02]
  -2.88577829430228E+02 .*        w_cv .* N9_cv  + ... % SE: 4.8042E+01, PE:  16.65%, 95% CI: [-3.8466E+02,-1.9249E+02]
  -1.95760234193630E+02 .*                 v_cv  + ... % SE: 4.0834E+01, PE:  20.86%, 95% CI: [-2.7743E+02,-1.1409E+02]
  +2.31693047620289E+02 .*       N1_cv .* N3_cv  + ... % SE: 4.8016E+01, PE:  20.72%, 95% CI: [+1.3566E+02,+3.2772E+02]
  -2.30536548911095E+02 .*        v_cv .* N2_cv  + ... % SE: 4.7636E+01, PE:  20.66%, 95% CI: [-3.2581E+02,-1.3527E+02]
  +2.19074256343953E+02 .*        v_cv .* N8_cv  + ... % SE: 4.7454E+01, PE:  21.66%, 95% CI: [+1.2417E+02,+3.1398E+02]
  -2.17800195656051E+02 .*       LE_cv .* N9_cv  + ... % SE: 4.7987E+01, PE:  22.03%, 95% CI: [-3.1377E+02,-1.2183E+02]
  +4.59779953706415E+02 .*        u_cv .* N9_cv  + ... % SE: 1.1868E+02, PE:  25.81%, 95% CI: [+2.2242E+02,+6.9714E+02]
  +3.68540874534542E+02 .*             N7_cv.^2  + ... % SE: 1.0402E+02, PE:  28.22%, 95% CI: [+1.6050E+02,+5.7658E+02]
  -1.69388920474513E+02 .*       RE_cv .* N9_cv  + ... % SE: 4.7426E+01, PE:  28.00%, 95% CI: [-2.6424E+02,-7.4536E+01]
  +1.68228441283830E+02 .*       N5_cv .* N7_cv  + ... % SE: 4.7599E+01, PE:  28.29%, 95% CI: [+7.3030E+01,+2.6343E+02]
  -1.65716896989613E+02 .*        v_cv .* LE_cv ;      % SE: 4.7428E+01, PE:  28.62%, 95% CI: [-2.6057E+02,-7.0861E+01]

% Myaw Model
RSQ(6)=95.726; PSE(6)=1.0583E+06; PRESS(6)=8.493E+08; NTERMS(6)=51;
Myaw = ...
  +4.07702239760010E+03 .*                 v_cv  + ... % SE: 1.5843E+02, PE:   3.89%, 95% CI: [+3.7602E+03,+4.3939E+03]
  -2.22153444030689E+03 .*                N3_cv  + ... % SE: 1.5694E+02, PE:   7.06%, 95% CI: [-2.5354E+03,-1.9077E+03]
  +1.86061385880203E+03 .*                N5_cv  + ... % SE: 4.3858E+01, PE:   2.36%, 95% CI: [+1.7729E+03,+1.9483E+03]
  -1.64904510907688E+03 .*                N6_cv  + ... % SE: 4.3764E+01, PE:   2.65%, 95% CI: [-1.7366E+03,-1.5615E+03]
  +1.55071507718023E+03 .*                N4_cv  + ... % SE: 4.3951E+01, PE:   2.83%, 95% CI: [+1.4628E+03,+1.6386E+03]
  +1.60541933563351E+03 .*        v_cv .* N9_cv  + ... % SE: 5.0157E+01, PE:   3.12%, 95% CI: [+1.5051E+03,+1.7057E+03]
  +1.21304531662361E+03 .*                N8_cv  + ... % SE: 4.3858E+01, PE:   3.62%, 95% CI: [+1.1253E+03,+1.3008E+03]
  -1.12553438049468E+03 .*                N2_cv  + ... % SE: 4.4043E+01, PE:   3.91%, 95% CI: [-1.2136E+03,-1.0374E+03]
  +1.02508375399055E+03 .*               RUD_cv  + ... % SE: 4.3864E+01, PE:   4.28%, 95% CI: [+9.3736E+02,+1.1128E+03]
  +1.10341933688825E+03 .*         v_cv .* w_cv  + ... % SE: 5.0550E+01, PE:   4.58%, 95% CI: [+1.0023E+03,+1.2045E+03]
  -6.41874235958960E+02 .*                N7_cv  + ... % SE: 4.3892E+01, PE:   6.84%, 95% CI: [-7.2966E+02,-5.5409E+02]
  +5.66787321347539E+02 .*                N1_cv  + ... % SE: 4.3795E+01, PE:   7.73%, 95% CI: [+4.7920E+02,+6.5438E+02]
  +6.84507774774186E+02 .*      RUD_cv .* N9_cv  + ... % SE: 5.0904E+01, PE:   7.44%, 95% CI: [+5.8270E+02,+7.8632E+02]
  -1.10187162900304E+03 .*         u_cv .* v_cv  + ... % SE: 1.2862E+02, PE:  11.67%, 95% CI: [-1.3591E+03,-8.4464E+02]
  +3.11102905716320E+02 .*        v_cv .* N8_cv  + ... % SE: 5.0353E+01, PE:  16.19%, 95% CI: [+2.1040E+02,+4.1181E+02]
  +3.48694979946010E+02 .*        v_cv .* N4_cv  + ... % SE: 5.0798E+01, PE:  14.57%, 95% CI: [+2.4710E+02,+4.5029E+02]
  +2.86808312468826E+02 .*        w_cv .* N7_cv  + ... % SE: 5.1142E+01, PE:  17.83%, 95% CI: [+1.8452E+02,+3.8909E+02]
  +3.50601633693007E+02 .*        w_cv .* N8_cv  + ... % SE: 5.1370E+01, PE:  14.65%, 95% CI: [+2.4786E+02,+4.5334E+02]
  -2.71783981899044E+02 .*             N9_cv.^3  + ... % SE: 4.7112E+01, PE:  17.33%, 95% CI: [-3.6601E+02,-1.7756E+02]
  -2.62195289600234E+02 .*        w_cv .* N1_cv  + ... % SE: 5.1055E+01, PE:  19.47%, 95% CI: [-3.6431E+02,-1.6009E+02]
  +2.76789077104231E+02 .*       N2_cv .* N4_cv  + ... % SE: 5.1016E+01, PE:  18.43%, 95% CI: [+1.7476E+02,+3.7882E+02]
  -2.56047963048409E+02 .*        w_cv .* N2_cv  + ... % SE: 5.1478E+01, PE:  20.11%, 95% CI: [-3.5900E+02,-1.5309E+02]
  +2.72818745102830E+02 .*                 u_cv  + ... % SE: 5.7433E+01, PE:  21.05%, 95% CI: [+1.5795E+02,+3.8768E+02]
  +2.38584598388338E+02 .*       w_cv .* RUD_cv  + ... % SE: 5.1230E+01, PE:  21.47%, 95% CI: [+1.3612E+02,+3.4104E+02]
  -7.63796908925470E+02 .*             N3_cv.^2  + ... % SE: 1.0574E+02, PE:  13.84%, 95% CI: [-9.7528E+02,-5.5232E+02]
  +6.96757148170278E+02 .*             N5_cv.^2  + ... % SE: 1.0678E+02, PE:  15.33%, 95% CI: [+4.8319E+02,+9.1032E+02]
  -6.36141667484144E+02 .*             N6_cv.^2  + ... % SE: 1.0738E+02, PE:  16.88%, 95% CI: [-8.5090E+02,-4.2139E+02]
  +5.45129647123521E+02 .*             N8_cv.^2  + ... % SE: 1.0734E+02, PE:  19.69%, 95% CI: [+3.3044E+02,+7.5982E+02]
  +2.08731035972128E+02 .*        v_cv .* N6_cv  + ... % SE: 5.0610E+01, PE:  24.25%, 95% CI: [+1.0751E+02,+3.0995E+02]
  +2.53148489892994E+02 .*              u_cv.^2  + ... % SE: 6.5268E+01, PE:  25.78%, 95% CI: [+1.2261E+02,+3.8368E+02]
  -4.52320036009265E+02 .*             N2_cv.^2  + ... % SE: 1.0766E+02, PE:  23.80%, 95% CI: [-6.6763E+02,-2.3701E+02]
  +4.56629195721058E+02 .*             N1_cv.^2  + ... % SE: 1.0682E+02, PE:  23.39%, 95% CI: [+2.4299E+02,+6.7027E+02]
  -1.93233146620514E+02 .*        v_cv .* N5_cv  + ... % SE: 5.0267E+01, PE:  26.01%, 95% CI: [-2.9377E+02,-9.2698E+01]
  -1.69353198418488E+02 .*        w_cv .* N4_cv  + ... % SE: 5.1426E+01, PE:  30.37%, 95% CI: [-2.7220E+02,-6.6502E+01]
  -1.53682664392159E+02 .*              w_cv.^3  + ... % SE: 4.7715E+01, PE:  31.05%, 95% CI: [-2.4911E+02,-5.8253E+01]
  +4.80720016114329E+02 .*        u_cv .* N6_cv  + ... % SE: 1.2743E+02, PE:  26.51%, 95% CI: [+2.2586E+02,+7.3558E+02]
  -1.51856198763877E+02 .*       N2_cv .* N3_cv  + ... % SE: 5.1139E+01, PE:  33.68%, 95% CI: [-2.5413E+02,-4.9579E+01]
  +1.56279865166867E+02 .*        v_cv .* N2_cv  + ... % SE: 5.0685E+01, PE:  32.43%, 95% CI: [+5.4910E+01,+2.5765E+02]
  -1.28458960290508E+02 .*      RUD_cv .* N8_cv  + ... % SE: 5.1208E+01, PE:  39.86%, 95% CI: [-2.3088E+02,-2.6043E+01]
  -4.21597514410363E+02 .*              v_cv.^3  + ... % SE: 1.7076E+02, PE:  40.50%, 95% CI: [-7.6312E+02,-8.0079E+01]
  -1.42807442859494E+02 .*        v_cv .* N3_cv  + ... % SE: 5.0244E+01, PE:  35.18%, 95% CI: [-2.4329E+02,-4.2320E+01]
  -1.26157906571573E+02 .*       N6_cv .* N8_cv  + ... % SE: 5.0866E+01, PE:  40.32%, 95% CI: [-2.2789E+02,-2.4425E+01]
  +1.27024978325260E+02 .*       LA_cv .* RA_cv  + ... % SE: 5.0715E+01, PE:  39.93%, 95% CI: [+2.5595E+01,+2.2846E+02]
  +1.34149685003108E+02 .*        w_cv .* N6_cv  + ... % SE: 5.1445E+01, PE:  38.35%, 95% CI: [+3.1260E+01,+2.3704E+02]
  +1.22531079989555E+02 .*      LE_cv .* RUD_cv  + ... % SE: 5.0886E+01, PE:  41.53%, 95% CI: [+2.0760E+01,+2.2430E+02]
  +1.11661125524413E+02 .*        v_cv .* LE_cv  + ... % SE: 5.0264E+01, PE:  45.01%, 95% CI: [+1.1133E+01,+2.1219E+02]
  +3.65410917441532E+02 .*             N3_cv.^3  + ... % SE: 1.6960E+02, PE:  46.41%, 95% CI: [+2.6214E+01,+7.0461E+02]
  -1.20960360186933E+02 .*       LE_cv .* N3_cv  + ... % SE: 5.1131E+01, PE:  42.27%, 95% CI: [-2.2322E+02,-1.8697E+01]
  -1.22820123280628E+02 .*       RA_cv .* N6_cv  + ... % SE: 5.1146E+01, PE:  41.64%, 95% CI: [-2.2511E+02,-2.0528E+01]
  +2.99612586780830E+02 .*         u_cv .* w_cv  + ... % SE: 1.3151E+02, PE:  43.89%, 95% CI: [+3.6588E+01,+5.6264E+02]
  -2.73828948247695E+02 .*        u_cv .* LA_cv ;      % SE: 1.2669E+02, PE:  46.27%, 95% CI: [-5.2720E+02,-2.0454E+01]

% Flift Model
RSQ(7)=92.928; PSE(7)=5.8632E+05; PRESS(7)=5.3667E+08; NTERMS(7)=45;
Flift = ...
  +7.11519925209520E+03 .*                    1  + ... % SE: 8.4955E+01, PE:   1.19%, 95% CI: [+6.9453E+03,+7.2851E+03]
  +3.20022723012618E+03 .*                 w_cv  + ... % SE: 1.2186E+02, PE:   3.81%, 95% CI: [+2.9565E+03,+3.4439E+03]
  -3.06751695797979E+03 .*              w_cv.^2  + ... % SE: 8.5395E+01, PE:   2.78%, 95% CI: [-3.2383E+03,-2.8967E+03]
  +1.37588514622534E+03 .*        w_cv .* N9_cv  + ... % SE: 3.9848E+01, PE:   2.90%, 95% CI: [+1.2962E+03,+1.4556E+03]
  +7.16275618917297E+02 .*                N8_cv  + ... % SE: 3.4334E+01, PE:   4.79%, 95% CI: [+6.4761E+02,+7.8494E+02]
  +2.96822338563379E+03 .*                 u_cv  + ... % SE: 1.2530E+02, PE:   4.22%, 95% CI: [+2.7176E+03,+3.2188E+03]
  +6.23181237984377E+02 .*                N2_cv  + ... % SE: 3.4354E+01, PE:   5.51%, 95% CI: [+5.5447E+02,+6.9189E+02]
  +8.57434973089651E+02 .*                N6_cv  + ... % SE: 1.2219E+02, PE:  14.25%, 95% CI: [+6.1306E+02,+1.1018E+03]
  +8.06612258445420E+02 .*                N4_cv  + ... % SE: 1.2162E+02, PE:  15.08%, 95% CI: [+5.6338E+02,+1.0498E+03]
  -2.60312618525862E+03 .*              u_cv.^3  + ... % SE: 1.4525E+02, PE:   5.58%, 95% CI: [-2.8936E+03,-2.3126E+03]
  +5.71088829085669E+02 .*                N7_cv  + ... % SE: 3.4223E+01, PE:   5.99%, 95% CI: [+5.0264E+02,+6.3953E+02]
  +5.51998193713121E+02 .*                N1_cv  + ... % SE: 3.4250E+01, PE:   6.20%, 95% CI: [+4.8350E+02,+6.2050E+02]
  +4.81742060366331E+02 .*                N3_cv  + ... % SE: 3.4352E+01, PE:   7.13%, 95% CI: [+4.1304E+02,+5.5045E+02]
  +7.09685719206459E+02 .*                N5_cv  + ... % SE: 1.2327E+02, PE:  17.37%, 95% CI: [+4.6314E+02,+9.5624E+02]
  +3.45733896599879E+02 .*             N4_cv.^2  + ... % SE: 8.6713E+01, PE:  25.08%, 95% CI: [+1.7231E+02,+5.1916E+02]
  -1.10035583832394E+03 .*              w_cv.^3  + ... % SE: 1.3198E+02, PE:  11.99%, 95% CI: [-1.3643E+03,-8.3640E+02]
  +7.38041721896917E+02 .*        u_cv .* N2_cv  + ... % SE: 1.0067E+02, PE:  13.64%, 95% CI: [+5.3671E+02,+9.3938E+02]
  +3.68103904597256E+02 .*             N1_cv.^2  + ... % SE: 8.7089E+01, PE:  23.66%, 95% CI: [+1.9392E+02,+5.4228E+02]
  -7.31550146164498E+02 .*         u_cv .* w_cv  + ... % SE: 1.0142E+02, PE:  13.86%, 95% CI: [-9.3440E+02,-5.2870E+02]
  +4.78255256688060E+02 .*        u_cv .* N6_cv  + ... % SE: 9.8747E+01, PE:  20.65%, 95% CI: [+2.8076E+02,+6.7575E+02]
  +3.27931233994440E+02 .*             N6_cv.^2  + ... % SE: 8.6980E+01, PE:  26.52%, 95% CI: [+1.5397E+02,+5.0189E+02]
  +4.21589392838525E+02 .*        u_cv .* N5_cv  + ... % SE: 1.0006E+02, PE:  23.73%, 95% CI: [+2.2146E+02,+6.2171E+02]
  -3.62252719538081E+02 .*         u_cv .* v_cv  + ... % SE: 1.0025E+02, PE:  27.67%, 95% CI: [-5.6275E+02,-1.6176E+02]
  +2.82450800307879E+02 .*             N8_cv.^2  + ... % SE: 8.6136E+01, PE:  30.50%, 95% CI: [+1.1018E+02,+4.5472E+02]
  +2.60781808071846E+02 .*        u_cv .* RA_cv  + ... % SE: 9.7478E+01, PE:  37.38%, 95% CI: [+6.5826E+01,+4.5574E+02]
  +2.67117248471076E+02 .*        u_cv .* N4_cv  + ... % SE: 9.9244E+01, PE:  37.15%, 95% CI: [+6.8630E+01,+4.6560E+02]
  +9.52418433459457E+01 .*                 v_cv  + ... % SE: 3.3878E+01, PE:  35.57%, 95% CI: [+2.7486E+01,+1.6300E+02]
  -1.12154852090677E+02 .*       RA_cv .* N9_cv  + ... % SE: 3.9503E+01, PE:  35.22%, 95% CI: [-1.9116E+02,-3.3148E+01]
  +3.16522494517747E+02 .*              u_cv.^2  + ... % SE: 9.6151E+01, PE:  30.38%, 95% CI: [+1.2422E+02,+5.0882E+02]
  +1.99878693015163E+02 .*             N3_cv.^2  + ... % SE: 8.6016E+01, PE:  43.03%, 95% CI: [+2.7847E+01,+3.7191E+02]
  +8.15775789259972E+01 .*             LE_cv.^3  + ... % SE: 3.6987E+01, PE:  45.34%, 95% CI: [+7.6041E+00,+1.5555E+02]
  +9.79494302496109E+01 .*       LA_cv .* RE_cv  + ... % SE: 3.9803E+01, PE:  40.64%, 95% CI: [+1.8344E+01,+1.7755E+02]
  +2.61152756942727E+02 .*        u_cv .* N3_cv  + ... % SE: 1.0161E+02, PE:  38.91%, 95% CI: [+5.7926E+01,+4.6438E+02]
  -8.84847731848790E+01 .*       N3_cv .* N4_cv  + ... % SE: 3.9919E+01, PE:  45.11%, 95% CI: [-1.6832E+02,-8.6471E+00]
  -7.52309171031324E+01 .*       N5_cv .* N8_cv  + ... % SE: 3.9948E+01, PE:  53.10%, 95% CI: [-1.5513E+02,+4.6653E+00]
  +2.19027099520231E+02 .*             N2_cv.^2  + ... % SE: 8.8473E+01, PE:  40.39%, 95% CI: [+4.2081E+01,+3.9597E+02]
  +2.28448735799335E+02 .*        u_cv .* N1_cv  + ... % SE: 1.0079E+02, PE:  44.12%, 95% CI: [+2.6874E+01,+4.3002E+02]
  -1.98070520394617E+02 .*             RE_cv.^2  + ... % SE: 8.9170E+01, PE:  45.02%, 95% CI: [-3.7641E+02,-1.9731E+01]
  +1.85654524070728E+02 .*       u_cv .* RUD_cv  + ... % SE: 9.9149E+01, PE:  53.41%, 95% CI: [-1.2644E+01,+3.8395E+02]
  -2.63487738977491E+02 .*             N5_cv.^3  + ... % SE: 1.3315E+02, PE:  50.53%, 95% CI: [-5.2978E+02,+2.8056E+00]
  +7.58646162251668E+01 .*        v_cv .* N1_cv  + ... % SE: 3.9437E+01, PE:  51.98%, 95% CI: [-3.0095E+00,+1.5474E+02]
  -2.50996328080759E+02 .*             N6_cv.^3  + ... % SE: 1.3217E+02, PE:  52.66%, 95% CI: [-5.1533E+02,+1.3335E+01]
  +1.78129499183661E+02 .*        u_cv .* N9_cv  + ... % SE: 9.9540E+01, PE:  55.88%, 95% CI: [-2.0950E+01,+3.7721E+02]
  -2.34480687653884E+02 .*             N4_cv.^3  + ... % SE: 1.3154E+02, PE:  56.10%, 95% CI: [-4.9756E+02,+2.8597E+01]
  -6.37621872750145E+01 .*             N9_cv.^3 ;      % SE: 3.6778E+01, PE:  57.68%, 95% CI: [-1.3732E+02,+9.7932E+00]

% Fdrag Model
RSQ(8)=98.326; PSE(8)=5.1451E+05; PRESS(8)=3.6082E+08; NTERMS(8)=24;
Fdrag = ...
  +8.55800699293879E+03 .*                 w_cv  + ... % SE: 1.0399E+02, PE:   1.22%, 95% CI: [+8.3500E+03,+8.7660E+03]
  -1.66586605655677E+03 .*                    1  + ... % SE: 6.1764E+01, PE:   3.71%, 95% CI: [-1.7894E+03,-1.5423E+03]
  -8.11197165848604E+02 .*                N9_cv  + ... % SE: 2.9220E+01, PE:   3.60%, 95% CI: [-8.6964E+02,-7.5276E+02]
  +6.50054854084849E+02 .*              v_cv.^2  + ... % SE: 6.8022E+01, PE:  10.46%, 95% CI: [+5.1401E+02,+7.8610E+02]
  -2.65533777706824E+03 .*              w_cv.^3  + ... % SE: 1.1260E+02, PE:   4.24%, 95% CI: [-2.8805E+03,-2.4301E+03]
  -1.77879537029975E+03 .*         u_cv .* w_cv  + ... % SE: 8.5020E+01, PE:   4.78%, 95% CI: [-1.9488E+03,-1.6088E+03]
  +6.91798773418984E+02 .*        w_cv .* N4_cv  + ... % SE: 3.4354E+01, PE:   4.97%, 95% CI: [+6.2309E+02,+7.6051E+02]
  +7.30045575601938E+02 .*        w_cv .* N8_cv  + ... % SE: 3.4237E+01, PE:   4.69%, 95% CI: [+6.6157E+02,+7.9852E+02]
  +6.63150180104475E+02 .*        w_cv .* N2_cv  + ... % SE: 3.4342E+01, PE:   5.18%, 95% CI: [+5.9447E+02,+7.3184E+02]
  +6.23014764761326E+02 .*        w_cv .* N6_cv  + ... % SE: 3.4188E+01, PE:   5.49%, 95% CI: [+5.5464E+02,+6.9139E+02]
  +5.72233395969038E+02 .*        w_cv .* N1_cv  + ... % SE: 3.4050E+01, PE:   5.95%, 95% CI: [+5.0413E+02,+6.4033E+02]
  +5.66582137035237E+02 .*        w_cv .* N7_cv  + ... % SE: 3.4043E+01, PE:   6.01%, 95% CI: [+4.9850E+02,+6.3467E+02]
  +5.51360246741245E+02 .*        w_cv .* N3_cv  + ... % SE: 3.4201E+01, PE:   6.20%, 95% CI: [+4.8296E+02,+6.1976E+02]
  +4.66897930771303E+02 .*        w_cv .* N5_cv  + ... % SE: 3.4109E+01, PE:   7.31%, 95% CI: [+3.9868E+02,+5.3512E+02]
  +6.29722168423681E+02 .*              w_cv.^2  + ... % SE: 6.6342E+01, PE:  10.54%, 95% CI: [+4.9704E+02,+7.6241E+02]
  -1.97467383846645E+02 .*        v_cv .* N5_cv  + ... % SE: 3.3594E+01, PE:  17.01%, 95% CI: [-2.6466E+02,-1.3028E+02]
  -1.14466079291185E+03 .*                 u_cv  + ... % SE: 1.0728E+02, PE:   9.37%, 95% CI: [-1.3592E+03,-9.3009E+02]
  +1.17905992191526E+03 .*              u_cv.^3  + ... % SE: 1.2430E+02, PE:  10.54%, 95% CI: [+9.3045E+02,+1.4277E+03]
  -4.72296694406156E+02 .*        u_cv .* N9_cv  + ... % SE: 8.4043E+01, PE:  17.79%, 95% CI: [-6.4038E+02,-3.0421E+02]
  -4.14125506207070E+02 .*              u_cv.^2  + ... % SE: 7.4091E+01, PE:  17.89%, 95% CI: [-5.6231E+02,-2.6594E+02]
  +1.43919145461662E+02 .*        v_cv .* N3_cv  + ... % SE: 3.3655E+01, PE:  23.38%, 95% CI: [+7.6609E+01,+2.1123E+02]
  +3.08997784743522E+02 .*         u_cv .* v_cv  + ... % SE: 8.5483E+01, PE:  27.66%, 95% CI: [+1.3803E+02,+4.7996E+02]
  -3.07514555262702E+02 .*        u_cv .* LE_cv  + ... % SE: 8.4280E+01, PE:  27.41%, 95% CI: [-4.7607E+02,-1.3896E+02]
  -1.20013707524424E+02 .*        v_cv .* N6_cv ;      % SE: 3.3728E+01, PE:  28.10%, 95% CI: [-1.8747E+02,-5.2558E+01]

% Modeling Metrics
Metrics.RSQ=RSQ;
Metrics.PSE=PSE;
Metrics.PRESS=PRESS;
Metrics.NTERMS=NTERMS;

C = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];

return