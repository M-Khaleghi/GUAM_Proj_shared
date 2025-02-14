function [C,Metrics]=LpC_Hover_R12t15_PCp2FI_SWR_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8)
% LpC_Hover_R12t15_PCp2FI_SWR_cv - Aerodynamic model for the LpC Hover
%
% DESCRIPTION: 
%   This script contains the aerodynamic model for LpC Hover. The script was
%   automatically generated using "GenModelCV.m" on 14-Jan-2021 07:59:54
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

%   Modeling Algorithm: SWR, Model Complexity: Quadratic+2FI+PC 
%   Model SF: 1, SWR Alpha: 0.9999 
%   Final Fval: [  15.32  15.31  15.32  15.32  15.32  15.32  15.32  15.31  ] 
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
%    BlockL2_R2_S2_B1_D7_output.dat  BlockL2_R2_S2_B2_D8_output.dat  BlockL2_R2_S2_B3_D9_output.dat  BlockL2_R2_S2_B4_D10_output.dat 
%
%   Explantory variable ranges used to develop model:
%             u: [     +20,     +45] kts,  Nord:  3
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
u_cv = ( u-(3.25000000000000E+01) )/(1.25000000000000E+01);
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
RSQ(1)=89.948; PSE(1)=4755.5; PRESS(1)=2.758E+06; NTERMS(1)=23;
Faxial = ...
  +1.73744725799018E+02 .*                 u_cv  + ... % SE: 2.9985E+00, PE:   1.73%, 95% CI: [+1.6775E+02,+1.7974E+02]
  -1.01095839846748E+02 .*                 w_cv  + ... % SE: 2.9917E+00, PE:   2.96%, 95% CI: [-1.0708E+02,-9.5112E+01]
  +3.68924779708020E+01 .*                N1_cv  + ... % SE: 3.0054E+00, PE:   8.15%, 95% CI: [+3.0882E+01,+4.2903E+01]
  +4.21446421535435E+01 .*                N2_cv  + ... % SE: 3.0243E+00, PE:   7.18%, 95% CI: [+3.6096E+01,+4.8193E+01]
  +3.54808024077899E+01 .*                N3_cv  + ... % SE: 3.0014E+00, PE:   8.46%, 95% CI: [+2.9478E+01,+4.1484E+01]
  +2.80283489572413E+01 .*                N4_cv  + ... % SE: 3.0299E+00, PE:  10.81%, 95% CI: [+2.1969E+01,+3.4088E+01]
  +3.56126570337142E+01 .*                N5_cv  + ... % SE: 3.0418E+00, PE:   8.54%, 95% CI: [+2.9529E+01,+4.1696E+01]
  +4.07110795794927E+01 .*                N8_cv  + ... % SE: 3.0217E+00, PE:   7.42%, 95% CI: [+3.4668E+01,+4.6755E+01]
  +3.92605823534057E+01 .*              u_cv.^2  + ... % SE: 6.9864E+00, PE:  17.79%, 95% CI: [+2.5288E+01,+5.3233E+01]
  -2.25471019929445E+01 .*         u_cv .* w_cv  + ... % SE: 3.4535E+00, PE:  15.32%, 95% CI: [-2.9454E+01,-1.5640E+01]
  -3.53429330724596E+01 .*              w_cv.^2  + ... % SE: 7.0036E+00, PE:  19.82%, 95% CI: [-4.9350E+01,-2.1336E+01]
  +4.25103877784231E+01 .*            RUD_cv.^2  + ... % SE: 7.0028E+00, PE:  16.47%, 95% CI: [+2.8505E+01,+5.6516E+01]
  +2.29639071142213E+01 .*        u_cv .* N3_cv  + ... % SE: 3.4609E+00, PE:  15.07%, 95% CI: [+1.6042E+01,+2.9886E+01]
  +1.38696321681731E+01 .*        v_cv .* N3_cv  + ... % SE: 3.4703E+00, PE:  25.02%, 95% CI: [+6.9291E+00,+2.0810E+01]
  +1.43947417889232E+01 .*        w_cv .* N3_cv  + ... % SE: 3.4511E+00, PE:  23.97%, 95% CI: [+7.4926E+00,+2.1297E+01]
  -2.46407006931824E+01 .*        v_cv .* N4_cv  + ... % SE: 3.5024E+00, PE:  14.21%, 95% CI: [-3.1645E+01,-1.7636E+01]
  -2.13460157368863E+01 .*        w_cv .* N4_cv  + ... % SE: 3.4852E+00, PE:  16.33%, 95% CI: [-2.8316E+01,-1.4376E+01]
  +2.08512452552624E+01 .*        u_cv .* N5_cv  + ... % SE: 3.5059E+00, PE:  16.81%, 95% CI: [+1.3839E+01,+2.7863E+01]
  +2.26423000693934E+01 .*        v_cv .* N6_cv  + ... % SE: 3.4799E+00, PE:  15.37%, 95% CI: [+1.5683E+01,+2.9602E+01]
  -1.95194881998253E+01 .*        w_cv .* N6_cv  + ... % SE: 3.4687E+00, PE:  17.77%, 95% CI: [-2.6457E+01,-1.2582E+01]
  +2.49603083818143E+01 .*             N6_cv.^3  + ... % SE: 3.2658E+00, PE:  13.08%, 95% CI: [+1.8429E+01,+3.1492E+01]
  +3.63786312136473E+01 .*             N7_cv.^3  + ... % SE: 3.2595E+00, PE:   8.96%, 95% CI: [+2.9860E+01,+4.2898E+01]
  +3.63820136726325E+02 .*                    1 ;      % SE: 4.4774E+00, PE:   1.23%, 95% CI: [+3.5487E+02,+3.7277E+02]

% Fside Model
RSQ(2)=94.799; PSE(2)=7343.4; PRESS(2)=3.8818E+06; NTERMS(2)=15;
Fside = ...
  -1.82228045788398E+02 .*                 v_cv  + ... % SE: 3.5993E+00, PE:   1.98%, 95% CI: [-1.8943E+02,-1.7503E+02]
  -4.98912483760464E+01 .*               RUD_cv  + ... % SE: 3.6210E+00, PE:   7.26%, 95% CI: [-5.7133E+01,-4.2649E+01]
  -1.86778733671800E+02 .*                N3_cv  + ... % SE: 3.5903E+00, PE:   1.92%, 95% CI: [-1.9396E+02,-1.7960E+02]
  -1.60373560041614E+02 .*                N4_cv  + ... % SE: 3.6296E+00, PE:   2.26%, 95% CI: [-1.6763E+02,-1.5311E+02]
  +1.90370228123492E+02 .*                N5_cv  + ... % SE: 3.6422E+00, PE:   1.91%, 95% CI: [+1.8309E+02,+1.9765E+02]
  +1.53162603559441E+02 .*                N6_cv  + ... % SE: 3.6153E+00, PE:   2.36%, 95% CI: [+1.4593E+02,+1.6039E+02]
  +3.63416957951606E+01 .*         u_cv .* v_cv  + ... % SE: 4.1401E+00, PE:  11.39%, 95% CI: [+2.8061E+01,+4.4622E+01]
  -2.44575651899569E+01 .*         v_cv .* w_cv  + ... % SE: 4.1393E+00, PE:  16.92%, 95% CI: [-3.2736E+01,-1.6179E+01]
  -2.96777345340105E+01 .*       u_cv .* RUD_cv  + ... % SE: 4.1674E+00, PE:  14.04%, 95% CI: [-3.8013E+01,-2.1343E+01]
  +2.23854309578754E+01 .*        w_cv .* N3_cv  + ... % SE: 4.1314E+00, PE:  18.46%, 95% CI: [+1.4123E+01,+3.0648E+01]
  -6.37850617995543E+01 .*             N3_cv.^2  + ... % SE: 7.7255E+00, PE:  12.11%, 95% CI: [-7.9236E+01,-4.8334E+01]
  -2.31690327458715E+01 .*        w_cv .* N5_cv  + ... % SE: 4.1808E+00, PE:  18.04%, 95% CI: [-3.1531E+01,-1.4807E+01]
  +6.81079276785307E+01 .*             N5_cv.^2  + ... % SE: 7.7239E+00, PE:  11.34%, 95% CI: [+5.2660E+01,+8.3556E+01]
  +2.09044199290087E+01 .*        u_cv .* N6_cv  + ... % SE: 4.1527E+00, PE:  19.87%, 95% CI: [+1.2599E+01,+2.9210E+01]
  -8.37936156839506E+00 .*                    1 ;      % SE: 5.1295E+00, PE:  61.22%, 95% CI: [-1.8638E+01,+1.8796E+00]

% Fnormal Model
RSQ(3)=99.204; PSE(3)=3.2973E+05; PRESS(3)=3.48E+07; NTERMS(3)=37;
Fnormal = ...
  +6.11000350923909E+02 .*                 u_cv  + ... % SE: 3.7833E+01, PE:   6.19%, 95% CI: [+5.3533E+02,+6.8667E+02]
  +1.19787464934786E+03 .*                 w_cv  + ... % SE: 1.0519E+01, PE:   0.88%, 95% CI: [+1.1768E+03,+1.2189E+03]
  +1.31249504133543E+02 .*                LA_cv  + ... % SE: 1.0599E+01, PE:   8.08%, 95% CI: [+1.1005E+02,+1.5245E+02]
  +1.23491764971045E+02 .*                RA_cv  + ... % SE: 1.0570E+01, PE:   8.56%, 95% CI: [+1.0235E+02,+1.4463E+02]
  +7.55947170978477E+02 .*                N1_cv  + ... % SE: 1.0557E+01, PE:   1.40%, 95% CI: [+7.3483E+02,+7.7706E+02]
  +1.11047595584060E+03 .*                N2_cv  + ... % SE: 1.0626E+01, PE:   0.96%, 95% CI: [+1.0892E+03,+1.1317E+03]
  +6.69168371086014E+02 .*                N3_cv  + ... % SE: 1.0575E+01, PE:   1.58%, 95% CI: [+6.4802E+02,+6.9032E+02]
  +1.17148671783706E+03 .*                N4_cv  + ... % SE: 1.0660E+01, PE:   0.91%, 95% CI: [+1.1502E+03,+1.1928E+03]
  +6.74064289694513E+02 .*                N5_cv  + ... % SE: 1.0721E+01, PE:   1.59%, 95% CI: [+6.5262E+02,+6.9551E+02]
  +1.18752061821171E+03 .*                N6_cv  + ... % SE: 1.0615E+01, PE:   0.89%, 95% CI: [+1.1663E+03,+1.2088E+03]
  +7.56365335228707E+02 .*                N7_cv  + ... % SE: 1.0634E+01, PE:   1.41%, 95% CI: [+7.3510E+02,+7.7763E+02]
  +1.10306706936763E+03 .*                N8_cv  + ... % SE: 1.0618E+01, PE:   0.96%, 95% CI: [+1.0818E+03,+1.1243E+03]
  -1.58618403406935E+02 .*              v_cv.^2  + ... % SE: 2.7521E+01, PE:  17.35%, 95% CI: [-2.1366E+02,-1.0358E+02]
  +3.08216392709191E+02 .*         u_cv .* w_cv  + ... % SE: 1.2143E+01, PE:   3.94%, 95% CI: [+2.8393E+02,+3.3250E+02]
  +1.80717946567280E+02 .*              w_cv.^2  + ... % SE: 2.7388E+01, PE:  15.16%, 95% CI: [+1.2594E+02,+2.3549E+02]
  +7.13786277884525E+01 .*        u_cv .* LA_cv  + ... % SE: 1.2195E+01, PE:  17.09%, 95% CI: [+4.6988E+01,+9.5769E+01]
  +7.32414732463290E+01 .*        u_cv .* RA_cv  + ... % SE: 1.2199E+01, PE:  16.66%, 95% CI: [+4.8843E+01,+9.7640E+01]
  +1.26329840532060E+02 .*             N1_cv.^2  + ... % SE: 2.7375E+01, PE:  21.67%, 95% CI: [+7.1579E+01,+1.8108E+02]
  +5.14685894099400E+01 .*        u_cv .* N2_cv  + ... % SE: 1.2271E+01, PE:  23.84%, 95% CI: [+2.6926E+01,+7.6011E+01]
  +8.92432366270363E+01 .*        w_cv .* N2_cv  + ... % SE: 1.2188E+01, PE:  13.66%, 95% CI: [+6.4867E+01,+1.1362E+02]
  +1.51990888739898E+02 .*             N2_cv.^2  + ... % SE: 2.6878E+01, PE:  17.68%, 95% CI: [+9.8234E+01,+2.0575E+02]
  +6.58549224366525E+01 .*        v_cv .* N3_cv  + ... % SE: 1.2226E+01, PE:  18.57%, 95% CI: [+4.1402E+01,+9.0308E+01]
  +1.74716514350309E+02 .*             N3_cv.^2  + ... % SE: 2.7111E+01, PE:  15.52%, 95% CI: [+1.2050E+02,+2.2894E+02]
  +1.05847485880460E+02 .*        u_cv .* N4_cv  + ... % SE: 1.2235E+01, PE:  11.56%, 95% CI: [+8.1378E+01,+1.3032E+02]
  +1.11053908272837E+02 .*        w_cv .* N4_cv  + ... % SE: 1.2292E+01, PE:  11.07%, 95% CI: [+8.6470E+01,+1.3564E+02]
  +1.31455729045219E+02 .*             N4_cv.^2  + ... % SE: 2.6757E+01, PE:  20.35%, 95% CI: [+7.7941E+01,+1.8497E+02]
  -6.06123776935591E+01 .*        v_cv .* N5_cv  + ... % SE: 1.2344E+01, PE:  20.37%, 95% CI: [-8.5300E+01,-3.5925E+01]
  +1.20024257641797E+02 .*             N5_cv.^2  + ... % SE: 2.6768E+01, PE:  22.30%, 95% CI: [+6.6489E+01,+1.7356E+02]
  +1.06611701518674E+02 .*        u_cv .* N6_cv  + ... % SE: 1.2218E+01, PE:  11.46%, 95% CI: [+8.2176E+01,+1.3105E+02]
  +1.17496730599139E+02 .*        w_cv .* N6_cv  + ... % SE: 1.2202E+01, PE:  10.39%, 95% CI: [+9.3093E+01,+1.4190E+02]
  +1.56778076150152E+02 .*             N6_cv.^2  + ... % SE: 2.7028E+01, PE:  17.24%, 95% CI: [+1.0272E+02,+2.1083E+02]
  +1.51276928268334E+02 .*             N7_cv.^2  + ... % SE: 2.7070E+01, PE:  17.89%, 95% CI: [+9.7138E+01,+2.0542E+02]
  +7.22628179183663E+01 .*        w_cv .* N8_cv  + ... % SE: 1.2239E+01, PE:  16.94%, 95% CI: [+4.7784E+01,+9.6741E+01]
  +5.74519577521466E+01 .*       N6_cv .* N8_cv  + ... % SE: 1.2331E+01, PE:  21.46%, 95% CI: [+3.2791E+01,+8.2113E+01]
  +1.58549894562800E+02 .*             N8_cv.^2  + ... % SE: 2.6977E+01, PE:  17.01%, 95% CI: [+1.0460E+02,+2.1250E+02]
  -1.82615348338147E+02 .*              u_cv.^3  + ... % SE: 4.0890E+01, PE:  22.39%, 95% CI: [-2.6440E+02,-1.0083E+02]
  +8.86856560417631E+03 .*                    1 ;      % SE: 1.6656E+01, PE:   0.19%, 95% CI: [+8.8353E+03,+8.9019E+03]

% Mroll Model
RSQ(4)=99.335; PSE(4)=3.5046E+07; PRESS(4)=4.045E+09; NTERMS(4)=27;
Mroll = ...
  -1.19024831796056E+03 .*                 v_cv  + ... % SE: 1.1455E+02, PE:   9.62%, 95% CI: [-1.4193E+03,-9.6116E+02]
  +1.73401495119765E+03 .*                LA_cv  + ... % SE: 1.1464E+02, PE:   6.61%, 95% CI: [+1.5047E+03,+1.9633E+03]
  -1.71457697442359E+03 .*                RA_cv  + ... % SE: 1.1464E+02, PE:   6.69%, 95% CI: [-1.9439E+03,-1.4853E+03]
  +1.37349339245889E+04 .*                N1_cv  + ... % SE: 1.1450E+02, PE:   0.83%, 95% CI: [+1.3506E+04,+1.3964E+04]
  +1.96355968863204E+04 .*                N2_cv  + ... % SE: 1.1517E+02, PE:   0.59%, 95% CI: [+1.9405E+04,+1.9866E+04]
  +4.67274363158291E+03 .*                N3_cv  + ... % SE: 1.1451E+02, PE:   2.45%, 95% CI: [+4.4437E+03,+4.9018E+03]
  +7.85796175751550E+03 .*                N4_cv  + ... % SE: 1.1551E+02, PE:   1.47%, 95% CI: [+7.6269E+03,+8.0890E+03]
  -4.67901822569237E+03 .*                N5_cv  + ... % SE: 1.1630E+02, PE:   2.49%, 95% CI: [-4.9116E+03,-4.4464E+03]
  -7.71343818814775E+03 .*                N6_cv  + ... % SE: 1.1508E+02, PE:   1.49%, 95% CI: [-7.9436E+03,-7.4833E+03]
  -1.37109789125638E+04 .*                N7_cv  + ... % SE: 1.1479E+02, PE:   0.84%, 95% CI: [-1.3941E+04,-1.3481E+04]
  -1.98816140887928E+04 .*                N8_cv  + ... % SE: 1.1500E+02, PE:   0.58%, 95% CI: [-2.0112E+04,-1.9652E+04]
  -1.70152193527522E+03 .*         v_cv .* w_cv  + ... % SE: 1.3217E+02, PE:   7.77%, 95% CI: [-1.9659E+03,-1.4372E+03]
  +9.68184866082652E+02 .*        u_cv .* LA_cv  + ... % SE: 1.3200E+02, PE:  13.63%, 95% CI: [+7.0418E+02,+1.2322E+03]
  -8.47099759240788E+02 .*        u_cv .* RA_cv  + ... % SE: 1.3236E+02, PE:  15.63%, 95% CI: [-1.1118E+03,-5.8238E+02]
  -1.05269009408010E+03 .*        w_cv .* N1_cv  + ... % SE: 1.3181E+02, PE:  12.52%, 95% CI: [-1.3163E+03,-7.8907E+02]
  +3.04863155241507E+03 .*             N1_cv.^2  + ... % SE: 2.9112E+02, PE:   9.55%, 95% CI: [+2.4664E+03,+3.6309E+03]
  +6.94547178282307E+02 .*        w_cv .* N2_cv  + ... % SE: 1.3213E+02, PE:  19.02%, 95% CI: [+4.3028E+02,+9.5881E+02]
  +3.66530936966196E+03 .*             N2_cv.^2  + ... % SE: 2.8402E+02, PE:   7.75%, 95% CI: [+3.0973E+03,+4.2334E+03]
  +1.69232045864354E+03 .*             N4_cv.^2  + ... % SE: 2.8382E+02, PE:  16.77%, 95% CI: [+1.1247E+03,+2.2600E+03]
  -1.12332991662306E+03 .*             N5_cv.^2  + ... % SE: 2.8411E+02, PE:  25.29%, 95% CI: [-1.6915E+03,-5.5511E+02]
  -1.40004660348821E+03 .*             N6_cv.^2  + ... % SE: 2.8707E+02, PE:  20.50%, 95% CI: [-1.9742E+03,-8.2591E+02]
  +5.56659612353026E+02 .*        v_cv .* N7_cv  + ... % SE: 1.3270E+02, PE:  23.84%, 95% CI: [+2.9125E+02,+8.2207E+02]
  +1.03847577435224E+03 .*        w_cv .* N7_cv  + ... % SE: 1.3215E+02, PE:  12.73%, 95% CI: [+7.7418E+02,+1.3028E+03]
  -2.79461235168912E+03 .*             N7_cv.^2  + ... % SE: 2.8755E+02, PE:  10.29%, 95% CI: [-3.3697E+03,-2.2195E+03]
  -8.94038122178927E+02 .*        w_cv .* N8_cv  + ... % SE: 1.3238E+02, PE:  14.81%, 95% CI: [-1.1588E+03,-6.2929E+02]
  -3.30468848588932E+03 .*             N8_cv.^2  + ... % SE: 2.8590E+02, PE:   8.65%, 95% CI: [-3.8765E+03,-2.7329E+03]
  +1.73030276070239E+02 .*                    1 ;      % SE: 1.7799E+02, PE: 102.86%, 95% CI: [-1.8294E+02,+5.2900E+02]

% Mpitch Model
RSQ(5)=99.081; PSE(5)=3.0205E+06; PRESS(5)=4.0085E+08; NTERMS(5)=32;
Mpitch = ...
  +1.66260721148895E+02 .*                 w_cv  + ... % SE: 3.5894E+01, PE:  21.59%, 95% CI: [+9.4473E+01,+2.3805E+02]
  -2.71500630124407E+02 .*                LE_cv  + ... % SE: 3.5976E+01, PE:  13.25%, 95% CI: [-3.4345E+02,-1.9955E+02]
  -3.27143789059607E+02 .*                RE_cv  + ... % SE: 3.6472E+01, PE:  11.15%, 95% CI: [-4.0009E+02,-2.5420E+02]
  +3.39600235198245E+03 .*                N1_cv  + ... % SE: 3.6022E+01, PE:   1.06%, 95% CI: [+3.3240E+03,+3.4680E+03]
  -3.41064172962081E+03 .*                N2_cv  + ... % SE: 3.6245E+01, PE:   1.06%, 95% CI: [-3.4831E+03,-3.3382E+03]
  +3.68020453439978E+03 .*                N3_cv  + ... % SE: 3.5954E+01, PE:   0.98%, 95% CI: [+3.6083E+03,+3.7521E+03]
  -2.88647630141527E+03 .*                N4_cv  + ... % SE: 3.6325E+01, PE:   1.26%, 95% CI: [-2.9591E+03,-2.8138E+03]
  +3.68736878898753E+03 .*                N5_cv  + ... % SE: 3.6444E+01, PE:   0.99%, 95% CI: [+3.6145E+03,+3.7603E+03]
  -2.92023255634149E+03 .*                N6_cv  + ... % SE: 3.6142E+01, PE:   1.24%, 95% CI: [-2.9925E+03,-2.8479E+03]
  +3.39359844574835E+03 .*                N7_cv  + ... % SE: 3.6171E+01, PE:   1.07%, 95% CI: [+3.3213E+03,+3.4659E+03]
  -3.37850942019789E+03 .*                N8_cv  + ... % SE: 3.6180E+01, PE:   1.07%, 95% CI: [-3.4509E+03,-3.3061E+03]
  -2.61587640467133E+02 .*         u_cv .* w_cv  + ... % SE: 4.1390E+01, PE:  15.82%, 95% CI: [-3.4437E+02,-1.7881E+02]
  +4.28290394172119E+02 .*              w_cv.^2  + ... % SE: 9.2725E+01, PE:  21.65%, 95% CI: [+2.4284E+02,+6.1374E+02]
  -2.18718362251414E+02 .*        u_cv .* LE_cv  + ... % SE: 4.1298E+01, PE:  18.88%, 95% CI: [-3.0131E+02,-1.3612E+02]
  -2.09214756703118E+02 .*        u_cv .* RE_cv  + ... % SE: 4.2071E+01, PE:  20.11%, 95% CI: [-2.9336E+02,-1.2507E+02]
  +5.49808623670123E+02 .*             N1_cv.^2  + ... % SE: 9.2442E+01, PE:  16.81%, 95% CI: [+3.6492E+02,+7.3469E+02]
  -8.29950450405909E+02 .*             N2_cv.^2  + ... % SE: 9.0934E+01, PE:  10.96%, 95% CI: [-1.0118E+03,-6.4808E+02]
  +1.68717236119434E+02 .*        u_cv .* N3_cv  + ... % SE: 4.1501E+01, PE:  24.60%, 95% CI: [+8.5715E+01,+2.5172E+02]
  +3.10429375220515E+02 .*        v_cv .* N3_cv  + ... % SE: 4.1565E+01, PE:  13.39%, 95% CI: [+2.2730E+02,+3.9356E+02]
  +3.34806259819019E+02 .*        w_cv .* N3_cv  + ... % SE: 4.1426E+01, PE:  12.37%, 95% CI: [+2.5195E+02,+4.1766E+02]
  +3.74963041973775E+02 .*             N3_cv.^2  + ... % SE: 9.1920E+01, PE:  24.51%, 95% CI: [+1.9112E+02,+5.5880E+02]
  -2.50844834443590E+02 .*        v_cv .* N4_cv  + ... % SE: 4.1981E+01, PE:  16.74%, 95% CI: [-3.3481E+02,-1.6688E+02]
  -6.75251058626562E+02 .*             N4_cv.^2  + ... % SE: 9.0871E+01, PE:  13.46%, 95% CI: [-8.5699E+02,-4.9351E+02]
  -2.82517518724622E+02 .*        v_cv .* N5_cv  + ... % SE: 4.2086E+01, PE:  14.90%, 95% CI: [-3.6669E+02,-1.9835E+02]
  +3.55160349658926E+02 .*        w_cv .* N5_cv  + ... % SE: 4.1878E+01, PE:  11.79%, 95% CI: [+2.7140E+02,+4.3892E+02]
  +5.27393412849728E+02 .*             N5_cv.^2  + ... % SE: 9.0624E+01, PE:  17.18%, 95% CI: [+3.4615E+02,+7.0864E+02]
  +1.99238821387056E+02 .*        v_cv .* N6_cv  + ... % SE: 4.1701E+01, PE:  20.93%, 95% CI: [+1.1584E+02,+2.8264E+02]
  -5.12976422629876E+02 .*             N6_cv.^2  + ... % SE: 9.1677E+01, PE:  17.87%, 95% CI: [-6.9633E+02,-3.2962E+02]
  +4.73575704841963E+02 .*             N7_cv.^2  + ... % SE: 9.1822E+01, PE:  19.39%, 95% CI: [+2.8993E+02,+6.5722E+02]
  -8.52140578762915E+02 .*             N8_cv.^2  + ... % SE: 9.1565E+01, PE:  10.75%, 95% CI: [-1.0353E+03,-6.6901E+02]
  +3.45762789912362E+02 .*              u_cv.^3  + ... % SE: 3.8831E+01, PE:  11.23%, 95% CI: [+2.6810E+02,+4.2343E+02]
  +2.86022768229662E+03 .*                    1 ;      % SE: 5.6621E+01, PE:   1.98%, 95% CI: [+2.7470E+03,+2.9735E+03]

% Myaw Model
RSQ(6)=96.694; PSE(6)=9.5546E+05; PRESS(6)=3.2877E+08; NTERMS(6)=30;
Myaw = ...
  -1.59353614829854E+02 .*                LA_cv  + ... % SE: 3.2657E+01, PE:  20.49%, 95% CI: [-2.2467E+02,-9.4039E+01]
  +7.41743116483286E+02 .*               RUD_cv  + ... % SE: 3.2907E+01, PE:   4.44%, 95% CI: [+6.7593E+02,+8.0756E+02]
  -1.48153297664354E+03 .*                N2_cv  + ... % SE: 3.2786E+01, PE:   2.21%, 95% CI: [-1.5471E+03,-1.4160E+03]
  -1.94342757657329E+03 .*                N3_cv  + ... % SE: 3.2581E+01, PE:   1.68%, 95% CI: [-2.0086E+03,-1.8783E+03]
  +1.95635211283511E+03 .*                N4_cv  + ... % SE: 3.2887E+01, PE:   1.68%, 95% CI: [+1.8906E+03,+2.0221E+03]
  +1.89225415601953E+03 .*                N5_cv  + ... % SE: 3.3046E+01, PE:   1.75%, 95% CI: [+1.8262E+03,+1.9583E+03]
  -1.91426019487890E+03 .*                N6_cv  + ... % SE: 3.2762E+01, PE:   1.71%, 95% CI: [-1.9798E+03,-1.8487E+03]
  +1.51391986601539E+03 .*                N8_cv  + ... % SE: 3.2776E+01, PE:   2.16%, 95% CI: [+1.4484E+03,+1.5795E+03]
  +1.94877659192514E+02 .*         v_cv .* w_cv  + ... % SE: 3.7575E+01, PE:  19.28%, 95% CI: [+1.1973E+02,+2.7003E+02]
  +3.99832798705285E+02 .*             RA_cv.^2  + ... % SE: 8.1348E+01, PE:  20.35%, 95% CI: [+2.3714E+02,+5.6253E+02]
  +5.45729093647823E+02 .*       u_cv .* RUD_cv  + ... % SE: 3.7797E+01, PE:   6.93%, 95% CI: [+4.7014E+02,+6.2132E+02]
  -2.02638601584544E+02 .*       w_cv .* RUD_cv  + ... % SE: 3.7760E+01, PE:  18.63%, 95% CI: [-2.7816E+02,-1.2712E+02]
  -2.77573146338670E+02 .*        u_cv .* N1_cv  + ... % SE: 3.7716E+01, PE:  13.59%, 95% CI: [-3.5300E+02,-2.0214E+02]
  -5.80467364902038E+02 .*             N2_cv.^2  + ... % SE: 8.0593E+01, PE:  13.88%, 95% CI: [-7.4165E+02,-4.1928E+02]
  -3.17404589334983E+02 .*        v_cv .* N3_cv  + ... % SE: 3.7664E+01, PE:  11.87%, 95% CI: [-3.9273E+02,-2.4208E+02]
  -5.78053387321655E+02 .*             N3_cv.^2  + ... % SE: 8.1088E+01, PE:  14.03%, 95% CI: [-7.4023E+02,-4.1588E+02]
  +3.66136804988473E+02 .*        u_cv .* N4_cv  + ... % SE: 3.7841E+01, PE:  10.34%, 95% CI: [+2.9045E+02,+4.4182E+02]
  +2.24962492428440E+02 .*        v_cv .* N4_cv  + ... % SE: 3.8007E+01, PE:  16.89%, 95% CI: [+1.4895E+02,+3.0098E+02]
  +2.63593609351347E+02 .*        w_cv .* N4_cv  + ... % SE: 3.7884E+01, PE:  14.37%, 95% CI: [+1.8783E+02,+3.3936E+02]
  -3.52412931533670E+02 .*        v_cv .* N5_cv  + ... % SE: 3.7993E+01, PE:  10.78%, 95% CI: [-4.2840E+02,-2.7643E+02]
  +4.40481274179831E+02 .*             N5_cv.^2  + ... % SE: 7.9786E+01, PE:  18.11%, 95% CI: [+2.8091E+02,+6.0005E+02]
  -2.82807794308067E+02 .*        u_cv .* N6_cv  + ... % SE: 3.7648E+01, PE:  13.31%, 95% CI: [-3.5810E+02,-2.0751E+02]
  +2.07870948806354E+02 .*        v_cv .* N6_cv  + ... % SE: 3.7797E+01, PE:  18.18%, 95% CI: [+1.3228E+02,+2.8347E+02]
  -2.10405721405006E+02 .*        w_cv .* N6_cv  + ... % SE: 3.7725E+01, PE:  17.93%, 95% CI: [-2.8586E+02,-1.3496E+02]
  +1.82427913378850E+02 .*       N5_cv .* N6_cv  + ... % SE: 3.8249E+01, PE:  20.97%, 95% CI: [+1.0593E+02,+2.5893E+02]
  -3.79118319369713E+02 .*             N6_cv.^2  + ... % SE: 8.0988E+01, PE:  21.36%, 95% CI: [-5.4109E+02,-2.1714E+02]
  +2.80737201128168E+02 .*        u_cv .* N7_cv  + ... % SE: 3.7740E+01, PE:  13.44%, 95% CI: [+2.0526E+02,+3.5622E+02]
  +1.70782023667846E+02 .*        u_cv .* N8_cv  + ... % SE: 3.7681E+01, PE:  22.06%, 95% CI: [+9.5421E+01,+2.4614E+02]
  +6.17421823232507E+02 .*             N8_cv.^2  + ... % SE: 8.0778E+01, PE:  13.08%, 95% CI: [+4.5586E+02,+7.7898E+02]
  +1.34572520964208E+02 .*                    1 ;      % SE: 5.0354E+01, PE:  37.42%, 95% CI: [+3.3864E+01,+2.3528E+02]

% Flift Model
RSQ(7)=99.063; PSE(7)=2.9553E+05; PRESS(7)=3.7489E+07; NTERMS(7)=35;
Flift = ...
  +7.23709823041182E+02 .*                 u_cv  + ... % SE: 1.0929E+01, PE:   1.51%, 95% CI: [+7.0185E+02,+7.4557E+02]
  +1.02784366475803E+03 .*                 w_cv  + ... % SE: 1.0915E+01, PE:   1.06%, 95% CI: [+1.0060E+03,+1.0497E+03]
  +1.29366458343621E+02 .*                LA_cv  + ... % SE: 1.0998E+01, PE:   8.50%, 95% CI: [+1.0737E+02,+1.5136E+02]
  +1.14729363132769E+02 .*                RA_cv  + ... % SE: 1.0974E+01, PE:   9.56%, 95% CI: [+9.2782E+01,+1.3668E+02]
  +7.20001330795837E+02 .*                N1_cv  + ... % SE: 1.0955E+01, PE:   1.52%, 95% CI: [+6.9809E+02,+7.4191E+02]
  +1.06757349978353E+03 .*                N2_cv  + ... % SE: 1.1026E+01, PE:   1.03%, 95% CI: [+1.0455E+03,+1.0896E+03]
  +6.34105021813161E+02 .*                N3_cv  + ... % SE: 1.0969E+01, PE:   1.73%, 95% CI: [+6.1217E+02,+6.5604E+02]
  +1.12441995423319E+03 .*                N4_cv  + ... % SE: 1.1062E+01, PE:   0.98%, 95% CI: [+1.1023E+03,+1.1465E+03]
  +6.38021280770436E+02 .*                N5_cv  + ... % SE: 1.1134E+01, PE:   1.75%, 95% CI: [+6.1575E+02,+6.6029E+02]
  +1.14209100305235E+03 .*                N6_cv  + ... % SE: 1.1012E+01, PE:   0.96%, 95% CI: [+1.1201E+03,+1.1641E+03]
  +7.21276770417925E+02 .*                N7_cv  + ... % SE: 1.1019E+01, PE:   1.53%, 95% CI: [+6.9924E+02,+7.4331E+02]
  +1.05072564310030E+03 .*                N8_cv  + ... % SE: 1.1017E+01, PE:   1.05%, 95% CI: [+1.0287E+03,+1.0728E+03]
  -1.46068132516029E+02 .*              v_cv.^2  + ... % SE: 2.8390E+01, PE:  19.44%, 95% CI: [-2.0285E+02,-8.9289E+01]
  +3.19305108278759E+02 .*         u_cv .* w_cv  + ... % SE: 1.2601E+01, PE:   3.95%, 95% CI: [+2.9410E+02,+3.4451E+02]
  -3.29482517428095E+02 .*              w_cv.^2  + ... % SE: 2.8249E+01, PE:   8.57%, 95% CI: [-3.8598E+02,-2.7298E+02]
  +7.10550360160283E+01 .*        u_cv .* LA_cv  + ... % SE: 1.2642E+01, PE:  17.79%, 95% CI: [+4.5771E+01,+9.6339E+01]
  +7.33316147041167E+01 .*        u_cv .* RA_cv  + ... % SE: 1.2660E+01, PE:  17.26%, 95% CI: [+4.8011E+01,+9.8652E+01]
  +1.12637077083347E+02 .*             N1_cv.^2  + ... % SE: 2.8153E+01, PE:  24.99%, 95% CI: [+5.6332E+01,+1.6894E+02]
  +8.32783103878522E+01 .*        u_cv .* N2_cv  + ... % SE: 1.2732E+01, PE:  15.29%, 95% CI: [+5.7815E+01,+1.0874E+02]
  +7.21394409414620E+01 .*        w_cv .* N2_cv  + ... % SE: 1.2649E+01, PE:  17.53%, 95% CI: [+4.6842E+01,+9.7437E+01]
  +1.70215564337175E+02 .*             N2_cv.^2  + ... % SE: 2.7748E+01, PE:  16.30%, 95% CI: [+1.1472E+02,+2.2571E+02]
  +6.88164356399954E+01 .*        v_cv .* N3_cv  + ... % SE: 1.2675E+01, PE:  18.42%, 95% CI: [+4.3467E+01,+9.4166E+01]
  +1.89748934039665E+02 .*             N3_cv.^2  + ... % SE: 2.8000E+01, PE:  14.76%, 95% CI: [+1.3375E+02,+2.4575E+02]
  +1.32762793064280E+02 .*        u_cv .* N4_cv  + ... % SE: 1.2691E+01, PE:   9.56%, 95% CI: [+1.0738E+02,+1.5814E+02]
  +9.81619282632940E+01 .*        w_cv .* N4_cv  + ... % SE: 1.2741E+01, PE:  12.98%, 95% CI: [+7.2680E+01,+1.2364E+02]
  +1.30982569365610E+02 .*             N4_cv.^2  + ... % SE: 2.7711E+01, PE:  21.16%, 95% CI: [+7.5560E+01,+1.8641E+02]
  -6.22448748049640E+01 .*        v_cv .* N5_cv  + ... % SE: 1.2798E+01, PE:  20.56%, 95% CI: [-8.7842E+01,-3.6648E+01]
  +1.35715781940332E+02 .*        u_cv .* N6_cv  + ... % SE: 1.2678E+01, PE:   9.34%, 95% CI: [+1.1036E+02,+1.6107E+02]
  +1.09654488482665E+02 .*        w_cv .* N6_cv  + ... % SE: 1.2651E+01, PE:  11.54%, 95% CI: [+8.4353E+01,+1.3496E+02]
  +1.26465511655780E+02 .*             N6_cv.^2  + ... % SE: 2.7926E+01, PE:  22.08%, 95% CI: [+7.0613E+01,+1.8232E+02]
  +1.90441419828849E+02 .*             N7_cv.^2  + ... % SE: 2.7948E+01, PE:  14.68%, 95% CI: [+1.3455E+02,+2.4634E+02]
  +7.97723091107771E+01 .*        u_cv .* N8_cv  + ... % SE: 1.2682E+01, PE:  15.90%, 95% CI: [+5.4408E+01,+1.0514E+02]
  +5.83213778762080E+01 .*        w_cv .* N8_cv  + ... % SE: 1.2692E+01, PE:  21.76%, 95% CI: [+3.2937E+01,+8.3706E+01]
  +1.38838815540519E+02 .*             N8_cv.^2  + ... % SE: 2.7892E+01, PE:  20.09%, 95% CI: [+8.3054E+01,+1.9462E+02]
  +8.90857603993581E+03 .*                    1 ;      % SE: 1.7242E+01, PE:   0.19%, 95% CI: [+8.8741E+03,+8.9431E+03]

% Fdrag Model
RSQ(8)=99.201; PSE(8)=1.9689E+05; PRESS(8)=3.4294E+07; NTERMS(8)=19;
Fdrag = ...
  +1.39541873058307E+02 .*                 u_cv  + ... % SE: 1.0575E+01, PE:   7.58%, 95% CI: [+1.1839E+02,+1.6069E+02]
  +2.90115537396532E+03 .*                 w_cv  + ... % SE: 1.0549E+01, PE:   0.36%, 95% CI: [+2.8801E+03,+2.9223E+03]
  +6.80123747428013E+01 .*                N2_cv  + ... % SE: 1.0660E+01, PE:  15.67%, 95% CI: [+4.6692E+01,+8.9332E+01]
  +4.30647162846966E+01 .*                N6_cv  + ... % SE: 1.0643E+01, PE:  24.71%, 95% CI: [+2.1778E+01,+6.4351E+01]
  -9.07583658308150E+02 .*         u_cv .* w_cv  + ... % SE: 1.2172E+01, PE:   1.34%, 95% CI: [-9.3193E+02,-8.8324E+02]
  +3.57953151552519E+02 .*              w_cv.^2  + ... % SE: 1.9367E+01, PE:   5.41%, 95% CI: [+3.1922E+02,+3.9669E+02]
  +2.45822314018921E+02 .*        w_cv .* N1_cv  + ... % SE: 1.2192E+01, PE:   4.96%, 95% CI: [+2.2144E+02,+2.7021E+02]
  +3.23435436033677E+02 .*        w_cv .* N2_cv  + ... % SE: 1.2220E+01, PE:   3.78%, 95% CI: [+2.9900E+02,+3.4788E+02]
  +9.51615085890313E+01 .*        v_cv .* N3_cv  + ... % SE: 1.2246E+01, PE:  12.87%, 95% CI: [+7.0669E+01,+1.1965E+02]
  +2.13669113591224E+02 .*        w_cv .* N3_cv  + ... % SE: 1.2173E+01, PE:   5.70%, 95% CI: [+1.8932E+02,+2.3802E+02]
  +3.40493242154840E+02 .*        w_cv .* N4_cv  + ... % SE: 1.2286E+01, PE:   3.61%, 95% CI: [+3.1592E+02,+3.6507E+02]
  -5.57741811486418E+01 .*        v_cv .* N5_cv  + ... % SE: 1.2355E+01, PE:  22.15%, 95% CI: [-8.0485E+01,-3.1064E+01]
  +2.17653656088659E+02 .*        w_cv .* N5_cv  + ... % SE: 1.2323E+01, PE:   5.66%, 95% CI: [+1.9301E+02,+2.4230E+02]
  +3.38941716600007E+02 .*        w_cv .* N6_cv  + ... % SE: 1.2227E+01, PE:   3.61%, 95% CI: [+3.1449E+02,+3.6340E+02]
  +2.29650528223171E+02 .*        w_cv .* N7_cv  + ... % SE: 1.2210E+01, PE:   5.32%, 95% CI: [+2.0523E+02,+2.5407E+02]
  +3.34318805550771E+02 .*        w_cv .* N8_cv  + ... % SE: 1.2240E+01, PE:   3.66%, 95% CI: [+3.0984E+02,+3.5880E+02]
  +6.00186392537917E+01 .*             N4_cv.^3  + ... % SE: 1.1598E+01, PE:  19.32%, 95% CI: [+3.6822E+01,+8.3215E+01]
  +7.24082241283718E+01 .*             N8_cv.^3  + ... % SE: 1.1542E+01, PE:  15.94%, 95% CI: [+4.9325E+01,+9.5492E+01]
  +3.95502240911123E+02 .*                    1 ;      % SE: 1.4037E+01, PE:   3.55%, 95% CI: [+3.6743E+02,+4.2358E+02]

% Modeling Metrics
Metrics.RSQ=RSQ;
Metrics.PSE=PSE;
Metrics.PRESS=PRESS;
Metrics.NTERMS=NTERMS;

C = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];

return