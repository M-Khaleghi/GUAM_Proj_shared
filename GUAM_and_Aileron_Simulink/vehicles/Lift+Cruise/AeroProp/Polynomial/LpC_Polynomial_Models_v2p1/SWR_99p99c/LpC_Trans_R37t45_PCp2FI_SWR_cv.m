function [C,Metrics]=LpC_Trans_R37t45_PCp2FI_SWR_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8,N9)
% LpC_Trans_R37t45_PCp2FI_SWR_cv - Aerodynamic model for the LpC Trans
%
% DESCRIPTION: 
%   This script contains the aerodynamic model for LpC Trans. The script was
%   automatically generated using "GenModelCV.m" on 13-Jan-2021 20:47:29
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

%   Modeling Algorithm: SWR, Model Complexity: Quadratic+2FI+PC 
%   Model SF: 1, SWR Alpha: 0.9999 
%   Final Fval: [  15.28  15.28  15.28  15.28  15.28  15.28  15.28  15.28  ] 
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
RSQ(1)=98.626; PSE(1)=50176; PRESS(1)=1.83E+07; NTERMS(1)=18;
Faxial = ...
  -6.79495248804220E+01 .*                 w_cv  + ... % SE: 6.6722E+00, PE:   9.82%, 95% CI: [-8.1294E+01,-5.4605E+01]
  -3.40190941529620E+01 .*                LE_cv  + ... % SE: 6.6372E+00, PE:  19.51%, 95% CI: [-4.7293E+01,-2.0745E+01]
  +5.06778933835106E+01 .*                RE_cv  + ... % SE: 6.6114E+00, PE:  13.05%, 95% CI: [+3.7455E+01,+6.3901E+01]
  +3.70255450121703E+01 .*               RUD_cv  + ... % SE: 6.6429E+00, PE:  17.94%, 95% CI: [+2.3740E+01,+5.0311E+01]
  +2.79212946490886E+01 .*                N6_cv  + ... % SE: 6.6370E+00, PE:  23.77%, 95% CI: [+1.4647E+01,+4.1195E+01]
  -1.61661798239485E+03 .*                N9_cv  + ... % SE: 6.6100E+00, PE:   0.41%, 95% CI: [-1.6298E+03,-1.6034E+03]
  -1.59691408003294E+02 .*              u_cv.^2  + ... % SE: 1.6749E+01, PE:  10.49%, 95% CI: [-1.9319E+02,-1.2619E+02]
  +1.47989632972951E+02 .*         u_cv .* v_cv  + ... % SE: 1.9281E+01, PE:  13.03%, 95% CI: [+1.0943E+02,+1.8655E+02]
  +6.79341037249652E+01 .*              v_cv.^2  + ... % SE: 1.5446E+01, PE:  22.74%, 95% CI: [+3.7042E+01,+9.8826E+01]
  -1.08194021533593E+02 .*         v_cv .* w_cv  + ... % SE: 7.6336E+00, PE:   7.06%, 95% CI: [-1.2346E+02,-9.2927E+01]
  -3.61091065519409E+01 .*        v_cv .* LE_cv  + ... % SE: 7.5994E+00, PE:  21.05%, 95% CI: [-5.1308E+01,-2.0910E+01]
  -4.91150292761957E+01 .*        w_cv .* N9_cv  + ... % SE: 7.6932E+00, PE:  15.66%, 95% CI: [-6.4501E+01,-3.3729E+01]
  +4.55985739463082E+01 .*       RE_cv .* N9_cv  + ... % SE: 7.6014E+00, PE:  16.67%, 95% CI: [+3.0396E+01,+6.0801E+01]
  -2.24588174841855E+02 .*             N9_cv.^2  + ... % SE: 1.5057E+01, PE:   6.70%, 95% CI: [-2.5470E+02,-1.9447E+02]
  +1.73512076075471E+02 .*              u_cv.^3  + ... % SE: 1.0089E+01, PE:   5.81%, 95% CI: [+1.5333E+02,+1.9369E+02]
  -3.09465837653419E+01 .*              v_cv.^3  + ... % SE: 7.0713E+00, PE:  22.85%, 95% CI: [-4.5089E+01,-1.6804E+01]
  +4.45127021062369E+01 .*             N4_cv.^3  + ... % SE: 7.1796E+00, PE:  16.13%, 95% CI: [+3.0153E+01,+5.8872E+01]
  -2.12757552107715E+03 .*                    1 ;      % SE: 1.4040E+01, PE:   0.66%, 95% CI: [-2.1557E+03,-2.0995E+03]

% Fside Model
RSQ(2)=96.861; PSE(2)=9569.2; PRESS(2)=5.1379E+06; NTERMS(2)=20;
Fside = ...
  -4.37711823256495E+02 .*                 v_cv  + ... % SE: 3.4523E+00, PE:   0.79%, 95% CI: [-4.4462E+02,-4.3081E+02]
  -6.35428251458488E+01 .*               RUD_cv  + ... % SE: 3.4979E+00, PE:   5.50%, 95% CI: [-7.0539E+01,-5.6547E+01]
  -1.74782725310176E+02 .*                N3_cv  + ... % SE: 3.4921E+00, PE:   2.00%, 95% CI: [-1.8177E+02,-1.6780E+02]
  -1.31878853118236E+02 .*                N4_cv  + ... % SE: 3.5000E+00, PE:   2.65%, 95% CI: [-1.3888E+02,-1.2488E+02]
  +1.79604590324596E+02 .*                N5_cv  + ... % SE: 3.4818E+00, PE:   1.94%, 95% CI: [+1.7264E+02,+1.8657E+02]
  +1.42278020482710E+02 .*                N6_cv  + ... % SE: 3.4927E+00, PE:   2.45%, 95% CI: [+1.3529E+02,+1.4926E+02]
  +1.59173006655043E+01 .*                N9_cv  + ... % SE: 3.4747E+00, PE:  21.83%, 95% CI: [+8.9678E+00,+2.2867E+01]
  +7.41210364992141E+01 .*         u_cv .* v_cv  + ... % SE: 1.0089E+01, PE:  13.61%, 95% CI: [+5.3943E+01,+9.4299E+01]
  -1.22182506525998E+02 .*         v_cv .* w_cv  + ... % SE: 4.0152E+00, PE:   3.29%, 95% CI: [-1.3021E+02,-1.1415E+02]
  -1.94394839774219E+01 .*        v_cv .* N1_cv  + ... % SE: 3.9990E+00, PE:  20.57%, 95% CI: [-2.7437E+01,-1.1442E+01]
  -3.54991263046818E+01 .*        v_cv .* N2_cv  + ... % SE: 4.0094E+00, PE:  11.29%, 95% CI: [-4.3518E+01,-2.7480E+01]
  -2.25587865682788E+01 .*        v_cv .* N3_cv  + ... % SE: 4.0000E+00, PE:  17.73%, 95% CI: [-3.0559E+01,-1.4559E+01]
  -3.72009049288117E+01 .*             N3_cv.^2  + ... % SE: 7.2921E+00, PE:  19.60%, 95% CI: [-5.1785E+01,-2.2617E+01]
  -1.98579772367946E+01 .*        v_cv .* N4_cv  + ... % SE: 4.0240E+00, PE:  20.26%, 95% CI: [-2.7906E+01,-1.1810E+01]
  -2.78029931468283E+01 .*        v_cv .* N5_cv  + ... % SE: 4.0033E+00, PE:  14.40%, 95% CI: [-3.5810E+01,-1.9796E+01]
  +4.75610385677551E+01 .*             N5_cv.^2  + ... % SE: 7.2900E+00, PE:  15.33%, 95% CI: [+3.2981E+01,+6.2141E+01]
  -4.01985941713938E+01 .*        v_cv .* N8_cv  + ... % SE: 4.0118E+00, PE:   9.98%, 95% CI: [-4.8222E+01,-3.2175E+01]
  -7.67985426070160E+01 .*        v_cv .* N9_cv  + ... % SE: 3.9816E+00, PE:   5.18%, 95% CI: [-8.4762E+01,-6.8835E+01]
  -4.58087024992578E+01 .*      RUD_cv .* N9_cv  + ... % SE: 4.0483E+00, PE:   8.84%, 95% CI: [-5.3905E+01,-3.7712E+01]
  +1.17233148724726E+01 .*                    1 ;      % SE: 4.3440E+00, PE:  37.05%, 95% CI: [+3.0353E+00,+2.0411E+01]

% Fnormal Model
RSQ(3)=99.066; PSE(3)=1.8447E+05; PRESS(3)=3.5303E+07; NTERMS(3)=33;
Fnormal = ...
  +4.42619251640403E+02 .*                 w_cv  + ... % SE: 9.0881E+00, PE:   2.05%, 95% CI: [+4.2444E+02,+4.6080E+02]
  +5.56584767715850E+01 .*                LE_cv  + ... % SE: 9.0381E+00, PE:  16.24%, 95% CI: [+3.7582E+01,+7.3735E+01]
  +8.31169802401708E+02 .*                N1_cv  + ... % SE: 9.0329E+00, PE:   1.09%, 95% CI: [+8.1310E+02,+8.4924E+02]
  +9.66762978074031E+02 .*                N2_cv  + ... % SE: 9.0758E+00, PE:   0.94%, 95% CI: [+9.4861E+02,+9.8491E+02]
  +7.79778693077883E+02 .*                N3_cv  + ... % SE: 9.0615E+00, PE:   1.16%, 95% CI: [+7.6166E+02,+7.9790E+02]
  +9.75514448445458E+02 .*                N4_cv  + ... % SE: 9.0683E+00, PE:   0.93%, 95% CI: [+9.5738E+02,+9.9365E+02]
  +7.76604971550543E+02 .*                N5_cv  + ... % SE: 9.0166E+00, PE:   1.16%, 95% CI: [+7.5857E+02,+7.9464E+02]
  +9.61003350406264E+02 .*                N6_cv  + ... % SE: 9.0472E+00, PE:   0.94%, 95% CI: [+9.4291E+02,+9.7910E+02]
  +8.27928429913691E+02 .*                N7_cv  + ... % SE: 9.0383E+00, PE:   1.09%, 95% CI: [+8.0985E+02,+8.4601E+02]
  +9.84452477855386E+02 .*                N8_cv  + ... % SE: 9.0722E+00, PE:   0.92%, 95% CI: [+9.6631E+02,+1.0026E+03]
  -6.77996486046251E+01 .*                N9_cv  + ... % SE: 9.0204E+00, PE:  13.30%, 95% CI: [-8.5840E+01,-4.9759E+01]
  +1.23407666873381E+02 .*         u_cv .* w_cv  + ... % SE: 2.6554E+01, PE:  21.52%, 95% CI: [+7.0300E+01,+1.7652E+02]
  +4.74871771542307E+01 .*       RA_cv .* RE_cv  + ... % SE: 1.0371E+01, PE:  21.84%, 95% CI: [+2.6745E+01,+6.8229E+01]
  -1.25004953258949E+02 .*             RE_cv.^2  + ... % SE: 2.3137E+01, PE:  18.51%, 95% CI: [-1.7128E+02,-7.8731E+01]
  -7.75226206146914E+01 .*        w_cv .* N1_cv  + ... % SE: 1.0509E+01, PE:  13.56%, 95% CI: [-9.8541E+01,-5.6504E+01]
  +2.10387576440892E+02 .*             N1_cv.^2  + ... % SE: 2.2828E+01, PE:  10.85%, 95% CI: [+1.6473E+02,+2.5604E+02]
  -5.10222187699166E+01 .*       N1_cv .* N2_cv  + ... % SE: 1.0512E+01, PE:  20.60%, 95% CI: [-7.2046E+01,-2.9998E+01]
  +2.16052443835547E+02 .*             N2_cv.^2  + ... % SE: 2.3073E+01, PE:  10.68%, 95% CI: [+1.6991E+02,+2.6220E+02]
  +7.98588863813122E+01 .*        v_cv .* N3_cv  + ... % SE: 1.0378E+01, PE:  13.00%, 95% CI: [+5.9102E+01,+1.0062E+02]
  -5.59016481625777E+01 .*        w_cv .* N3_cv  + ... % SE: 1.0550E+01, PE:  18.87%, 95% CI: [-7.7001E+01,-3.4802E+01]
  +1.91493326197165E+02 .*             N3_cv.^2  + ... % SE: 2.2664E+01, PE:  11.84%, 95% CI: [+1.4616E+02,+2.3682E+02]
  +2.62993766549326E+02 .*             N4_cv.^2  + ... % SE: 2.2857E+01, PE:   8.69%, 95% CI: [+2.1728E+02,+3.0871E+02]
  -8.93108070097156E+01 .*        v_cv .* N5_cv  + ... % SE: 1.0352E+01, PE:  11.59%, 95% CI: [-1.1001E+02,-6.8607E+01]
  -5.84210933353978E+01 .*        w_cv .* N5_cv  + ... % SE: 1.0541E+01, PE:  18.04%, 95% CI: [-7.9504E+01,-3.7338E+01]
  +1.68122391589026E+02 .*             N5_cv.^2  + ... % SE: 2.2743E+01, PE:  13.53%, 95% CI: [+1.2264E+02,+2.1361E+02]
  +2.34122538309330E+02 .*             N6_cv.^2  + ... % SE: 2.2879E+01, PE:   9.77%, 95% CI: [+1.8836E+02,+2.7988E+02]
  -5.54450229793493E+01 .*        w_cv .* N7_cv  + ... % SE: 1.0518E+01, PE:  18.97%, 95% CI: [-7.6482E+01,-3.4408E+01]
  +4.80060217422221E+01 .*       N6_cv .* N7_cv  + ... % SE: 1.0489E+01, PE:  21.85%, 95% CI: [+2.7027E+01,+6.8985E+01]
  +2.53313238729039E+02 .*             N7_cv.^2  + ... % SE: 2.2786E+01, PE:   9.00%, 95% CI: [+2.0774E+02,+2.9889E+02]
  +2.19724282913928E+02 .*             N8_cv.^2  + ... % SE: 2.2760E+01, PE:  10.36%, 95% CI: [+1.7420E+02,+2.6524E+02]
  +4.86285646368559E+01 .*        w_cv .* N9_cv  + ... % SE: 1.0514E+01, PE:  21.62%, 95% CI: [+2.7601E+01,+6.9656E+01]
  +5.76619466817479E+01 .*             RE_cv.^3  + ... % SE: 9.6998E+00, PE:  16.82%, 95% CI: [+3.8262E+01,+7.7061E+01]
  +7.73869789129116E+03 .*                    1 ;      % SE: 1.2078E+01, PE:   0.16%, 95% CI: [+7.7145E+03,+7.7629E+03]

% Mroll Model
RSQ(4)=99.414; PSE(4)=3.3068E+07; PRESS(4)=4.4553E+09; NTERMS(4)=31;
Mroll = ...
  -5.71311225786500E+03 .*                 v_cv  + ... % SE: 1.0113E+02, PE:   1.77%, 95% CI: [-5.9154E+03,-5.5108E+03]
  -5.74554561257328E+02 .*                RA_cv  + ... % SE: 1.0202E+02, PE:  17.76%, 95% CI: [-7.7860E+02,-3.7051E+02]
  +1.57792951864512E+04 .*                N1_cv  + ... % SE: 1.0217E+02, PE:   0.65%, 95% CI: [+1.5575E+04,+1.5984E+04]
  +1.85579506956910E+04 .*                N2_cv  + ... % SE: 1.0270E+02, PE:   0.55%, 95% CI: [+1.8353E+04,+1.8763E+04]
  +6.89746946817126E+03 .*                N3_cv  + ... % SE: 1.0233E+02, PE:   1.48%, 95% CI: [+6.6928E+03,+7.1021E+03]
  +7.73338404268938E+03 .*                N4_cv  + ... % SE: 1.0252E+02, PE:   1.33%, 95% CI: [+7.5283E+03,+7.9384E+03]
  -6.75195559577250E+03 .*                N5_cv  + ... % SE: 1.0197E+02, PE:   1.51%, 95% CI: [-6.9559E+03,-6.5480E+03]
  -7.42383319079370E+03 .*                N6_cv  + ... % SE: 1.0233E+02, PE:   1.38%, 95% CI: [-7.6285E+03,-7.2192E+03]
  -1.56362830936651E+04 .*                N7_cv  + ... % SE: 1.0228E+02, PE:   0.65%, 95% CI: [-1.5841E+04,-1.5432E+04]
  -1.84913495243668E+04 .*                N8_cv  + ... % SE: 1.0234E+02, PE:   0.55%, 95% CI: [-1.8696E+04,-1.8287E+04]
  -2.38373765874725E+03 .*                N9_cv  + ... % SE: 1.0197E+02, PE:   4.28%, 95% CI: [-2.5877E+03,-2.1798E+03]
  +2.09539919888705E+03 .*         u_cv .* v_cv  + ... % SE: 2.9783E+02, PE:  14.21%, 95% CI: [+1.4997E+03,+2.6911E+03]
  -5.24829696962926E+03 .*         v_cv .* w_cv  + ... % SE: 1.1786E+02, PE:   2.25%, 95% CI: [-5.4840E+03,-5.0126E+03]
  -1.75738666223779E+03 .*        u_cv .* N1_cv  + ... % SE: 2.9937E+02, PE:  17.04%, 95% CI: [-2.3561E+03,-1.1586E+03]
  -9.60882748335831E+02 .*        w_cv .* N1_cv  + ... % SE: 1.1900E+02, PE:  12.38%, 95% CI: [-1.1989E+03,-7.2288E+02]
  +4.03906703119203E+03 .*             N1_cv.^2  + ... % SE: 2.5567E+02, PE:   6.33%, 95% CI: [+3.5277E+03,+4.5504E+03]
  +5.06837714303524E+02 .*        w_cv .* N2_cv  + ... % SE: 1.2006E+02, PE:  23.69%, 95% CI: [+2.6672E+02,+7.4696E+02]
  -4.89407915304143E+02 .*       N1_cv .* N2_cv  + ... % SE: 1.1918E+02, PE:  24.35%, 95% CI: [-7.2776E+02,-2.5105E+02]
  +3.42092998799498E+03 .*             N2_cv.^2  + ... % SE: 2.5587E+02, PE:   7.48%, 95% CI: [+2.9092E+03,+3.9327E+03]
  +1.80540505199226E+03 .*             N3_cv.^2  + ... % SE: 2.5472E+02, PE:  14.11%, 95% CI: [+1.2960E+03,+2.3148E+03]
  -8.06499857796667E+02 .*        v_cv .* N4_cv  + ... % SE: 1.1776E+02, PE:  14.60%, 95% CI: [-1.0420E+03,-5.7098E+02]
  +2.17138163522104E+03 .*             N4_cv.^2  + ... % SE: 2.5634E+02, PE:  11.81%, 95% CI: [+1.6587E+03,+2.6841E+03]
  -1.72878988503643E+03 .*             N5_cv.^2  + ... % SE: 2.5583E+02, PE:  14.80%, 95% CI: [-2.2404E+03,-1.2171E+03]
  -6.73493568369837E+02 .*        v_cv .* N6_cv  + ... % SE: 1.1728E+02, PE:  17.41%, 95% CI: [-9.0805E+02,-4.3894E+02]
  -2.05997076433415E+03 .*             N6_cv.^2  + ... % SE: 2.5634E+02, PE:  12.44%, 95% CI: [-2.5726E+03,-1.5473E+03]
  +1.02848938094288E+03 .*        w_cv .* N7_cv  + ... % SE: 1.1910E+02, PE:  11.58%, 95% CI: [+7.9029E+02,+1.2667E+03]
  -3.79415502349025E+03 .*             N7_cv.^2  + ... % SE: 2.5647E+02, PE:   6.76%, 95% CI: [-4.3071E+03,-3.2812E+03]
  -4.11410551601583E+03 .*             N8_cv.^2  + ... % SE: 2.5608E+02, PE:   6.22%, 95% CI: [-4.6263E+03,-3.6019E+03]
  -5.17343311655402E+02 .*        v_cv .* N9_cv  + ... % SE: 1.1705E+02, PE:  22.62%, 95% CI: [-7.5144E+02,-2.8325E+02]
  +5.10293766616588E+02 .*             LA_cv.^3  + ... % SE: 1.1033E+02, PE:  21.62%, 95% CI: [+2.8963E+02,+7.3096E+02]
  -2.77337916306049E+03 .*                    1 ;      % SE: 1.3613E+02, PE:   4.91%, 95% CI: [-3.0456E+03,-2.5011E+03]

% Mpitch Model
RSQ(5)=98.282; PSE(5)=2.5625E+06; PRESS(5)=7.8411E+08; NTERMS(5)=32;
Mpitch = ...
  +1.51658353945695E+03 .*                 u_cv  + ... % SE: 5.6193E+01, PE:   3.71%, 95% CI: [+1.4042E+03,+1.6290E+03]
  -1.94189700501648E+02 .*                 v_cv  + ... % SE: 4.2277E+01, PE:  21.77%, 95% CI: [-2.7874E+02,-1.0964E+02]
  -4.57162404538821E+02 .*                LE_cv  + ... % SE: 4.2755E+01, PE:   9.35%, 95% CI: [-5.4267E+02,-3.7165E+02]
  -2.45162893722918E+02 .*                RE_cv  + ... % SE: 4.2455E+01, PE:  17.32%, 95% CI: [-3.3007E+02,-1.6025E+02]
  +3.19797039709051E+03 .*                N1_cv  + ... % SE: 4.2690E+01, PE:   1.33%, 95% CI: [+3.1126E+03,+3.2834E+03]
  -3.34247925112107E+03 .*                N2_cv  + ... % SE: 4.2918E+01, PE:   1.28%, 95% CI: [-3.4283E+03,-3.2566E+03]
  +3.24839160000333E+03 .*                N3_cv  + ... % SE: 4.2781E+01, PE:   1.32%, 95% CI: [+3.1628E+03,+3.3340E+03]
  -3.00913495203842E+03 .*                N4_cv  + ... % SE: 4.2857E+01, PE:   1.42%, 95% CI: [-3.0948E+03,-2.9234E+03]
  +3.20045768261595E+03 .*                N5_cv  + ... % SE: 4.2661E+01, PE:   1.33%, 95% CI: [+3.1151E+03,+3.2858E+03]
  -2.96428154452188E+03 .*                N6_cv  + ... % SE: 4.2797E+01, PE:   1.44%, 95% CI: [-3.0499E+03,-2.8787E+03]
  +3.27608798732794E+03 .*                N7_cv  + ... % SE: 4.2693E+01, PE:   1.30%, 95% CI: [+3.1907E+03,+3.3615E+03]
  -3.34847353635835E+03 .*                N8_cv  + ... % SE: 4.2838E+01, PE:   1.28%, 95% CI: [-3.4341E+03,-3.2628E+03]
  -1.97441143143224E+03 .*                N9_cv  + ... % SE: 4.2584E+01, PE:   2.16%, 95% CI: [-2.0596E+03,-1.8892E+03]
  +1.25738621546061E+03 .*         u_cv .* w_cv  + ... % SE: 1.2600E+02, PE:  10.02%, 95% CI: [+1.0054E+03,+1.5094E+03]
  -2.84110770280979E+02 .*         v_cv .* w_cv  + ... % SE: 4.9291E+01, PE:  17.35%, 95% CI: [-3.8269E+02,-1.8553E+02]
  +7.22619142390010E+02 .*              w_cv.^2  + ... % SE: 1.0539E+02, PE:  14.58%, 95% CI: [+5.1184E+02,+9.3340E+02]
  +6.80784332282663E+02 .*             N1_cv.^2  + ... % SE: 1.0721E+02, PE:  15.75%, 95% CI: [+4.6636E+02,+8.9521E+02]
  -2.24958573256391E+02 .*        v_cv .* N2_cv  + ... % SE: 4.9243E+01, PE:  21.89%, 95% CI: [-3.2344E+02,-1.2647E+02]
  -6.76834463278452E+02 .*             N2_cv.^2  + ... % SE: 1.0798E+02, PE:  15.95%, 95% CI: [-8.9280E+02,-4.6087E+02]
  +4.95603227265234E+02 .*        v_cv .* N3_cv  + ... % SE: 4.9047E+01, PE:   9.90%, 95% CI: [+3.9751E+02,+5.9370E+02]
  +2.41643789413471E+02 .*       N1_cv .* N3_cv  + ... % SE: 4.9626E+01, PE:  20.54%, 95% CI: [+1.4239E+02,+3.4089E+02]
  +7.22634977779330E+02 .*             N3_cv.^2  + ... % SE: 1.0629E+02, PE:  14.71%, 95% CI: [+5.1005E+02,+9.3522E+02]
  -9.57223611317024E+02 .*             N4_cv.^2  + ... % SE: 1.0688E+02, PE:  11.17%, 95% CI: [-1.1710E+03,-7.4346E+02]
  -5.29265799365247E+02 .*        v_cv .* N5_cv  + ... % SE: 4.8926E+01, PE:   9.24%, 95% CI: [-6.2712E+02,-4.3141E+02]
  +6.42510530374594E+02 .*             N5_cv.^2  + ... % SE: 1.0700E+02, PE:  16.65%, 95% CI: [+4.2851E+02,+8.5651E+02]
  -9.55474009541459E+02 .*             N6_cv.^2  + ... % SE: 1.0747E+02, PE:  11.25%, 95% CI: [-1.1704E+03,-7.4054E+02]
  +2.09164089206305E+02 .*        v_cv .* N8_cv  + ... % SE: 4.9098E+01, PE:  23.47%, 95% CI: [+1.1097E+02,+3.0736E+02]
  -9.87791380394749E+02 .*             N8_cv.^2  + ... % SE: 1.0695E+02, PE:  10.83%, 95% CI: [-1.2017E+03,-7.7390E+02]
  -2.89104390665844E+02 .*        w_cv .* N9_cv  + ... % SE: 4.9711E+01, PE:  17.19%, 95% CI: [-3.8853E+02,-1.8968E+02]
  -2.11184486108268E+02 .*       LE_cv .* N9_cv  + ... % SE: 4.9629E+01, PE:  23.50%, 95% CI: [-3.1044E+02,-1.1193E+02]
  -5.05450509173112E+02 .*              w_cv.^3  + ... % SE: 4.6576E+01, PE:   9.21%, 95% CI: [-5.9860E+02,-4.1230E+02]
  -1.13822283444164E+03 .*                    1 ;      % SE: 5.6944E+01, PE:   5.00%, 95% CI: [-1.2521E+03,-1.0243E+03]

% Myaw Model
RSQ(6)=94.85; PSE(6)=1.6715E+06; PRESS(6)=9.5632E+08; NTERMS(6)=30;
Myaw = ...
  +2.78203238156514E+02 .*                 u_cv  + ... % SE: 6.2132E+01, PE:  22.33%, 95% CI: [+1.5394E+02,+4.0247E+02]
  +3.69696200506569E+03 .*                 v_cv  + ... % SE: 4.6796E+01, PE:   1.27%, 95% CI: [+3.6034E+03,+3.7906E+03]
  +1.04687718000552E+03 .*               RUD_cv  + ... % SE: 4.7392E+01, PE:   4.53%, 95% CI: [+9.5209E+02,+1.1417E+03]
  +5.84999112393412E+02 .*                N1_cv  + ... % SE: 4.7267E+01, PE:   8.08%, 95% CI: [+4.9046E+02,+6.7953E+02]
  -1.12465362677963E+03 .*                N2_cv  + ... % SE: 4.7515E+01, PE:   4.22%, 95% CI: [-1.2197E+03,-1.0296E+03]
  -1.88947433967242E+03 .*                N3_cv  + ... % SE: 4.7390E+01, PE:   2.51%, 95% CI: [-1.9843E+03,-1.7947E+03]
  +1.56411995751673E+03 .*                N4_cv  + ... % SE: 4.7451E+01, PE:   3.03%, 95% CI: [+1.4692E+03,+1.6590E+03]
  +1.84261562358468E+03 .*                N5_cv  + ... % SE: 4.7287E+01, PE:   2.57%, 95% CI: [+1.7480E+03,+1.9372E+03]
  -1.63673343430014E+03 .*                N6_cv  + ... % SE: 4.7314E+01, PE:   2.89%, 95% CI: [-1.7314E+03,-1.5421E+03]
  -6.31647593143457E+02 .*                N7_cv  + ... % SE: 4.7311E+01, PE:   7.49%, 95% CI: [-7.2627E+02,-5.3703E+02]
  +1.21164739671506E+03 .*                N8_cv  + ... % SE: 4.7354E+01, PE:   3.91%, 95% CI: [+1.1169E+03,+1.3064E+03]
  -1.10936165176765E+03 .*         u_cv .* v_cv  + ... % SE: 1.3730E+02, PE:  12.38%, 95% CI: [-1.3840E+03,-8.3475E+02]
  +1.09768651736518E+03 .*         v_cv .* w_cv  + ... % SE: 5.4429E+01, PE:   4.96%, 95% CI: [+9.8883E+02,+1.2065E+03]
  +2.24819765343052E+02 .*       w_cv .* RUD_cv  + ... % SE: 5.5326E+01, PE:  24.61%, 95% CI: [+1.1417E+02,+3.3547E+02]
  -2.72351352114359E+02 .*        w_cv .* N1_cv  + ... % SE: 5.5060E+01, PE:  20.22%, 95% CI: [-3.8247E+02,-1.6223E+02]
  -2.82137905384466E+02 .*        w_cv .* N2_cv  + ... % SE: 5.5578E+01, PE:  19.70%, 95% CI: [-3.9329E+02,-1.7098E+02]
  -4.49538302840730E+02 .*             N2_cv.^2  + ... % SE: 1.1439E+02, PE:  25.45%, 95% CI: [-6.7832E+02,-2.2076E+02]
  -6.73792337170341E+02 .*             N3_cv.^2  + ... % SE: 1.1334E+02, PE:  16.82%, 95% CI: [-9.0047E+02,-4.4712E+02]
  +3.34834025767998E+02 .*        v_cv .* N4_cv  + ... % SE: 5.4563E+01, PE:  16.30%, 95% CI: [+2.2571E+02,+4.4396E+02]
  +2.97557011874070E+02 .*       N2_cv .* N4_cv  + ... % SE: 5.4991E+01, PE:  18.48%, 95% CI: [+1.8757E+02,+4.0754E+02]
  +6.62717682730295E+02 .*             N5_cv.^2  + ... % SE: 1.1381E+02, PE:  17.17%, 95% CI: [+4.3509E+02,+8.9035E+02]
  -5.49025513585067E+02 .*             N6_cv.^2  + ... % SE: 1.1452E+02, PE:  20.86%, 95% CI: [-7.7806E+02,-3.1999E+02]
  +2.88332793306715E+02 .*        w_cv .* N7_cv  + ... % SE: 5.5057E+01, PE:  19.10%, 95% CI: [+1.7822E+02,+3.9845E+02]
  +3.18382083739887E+02 .*        v_cv .* N8_cv  + ... % SE: 5.4409E+01, PE:  17.09%, 95% CI: [+2.0956E+02,+4.2720E+02]
  +3.30510501924336E+02 .*        w_cv .* N8_cv  + ... % SE: 5.5342E+01, PE:  16.74%, 95% CI: [+2.1983E+02,+4.4119E+02]
  +5.63721628689812E+02 .*             N8_cv.^2  + ... % SE: 1.1434E+02, PE:  20.28%, 95% CI: [+3.3505E+02,+7.9239E+02]
  +1.61915465840308E+03 .*        v_cv .* N9_cv  + ... % SE: 5.4103E+01, PE:   3.34%, 95% CI: [+1.5109E+03,+1.7274E+03]
  +6.62360122467267E+02 .*      RUD_cv .* N9_cv  + ... % SE: 5.4966E+01, PE:   8.30%, 95% CI: [+5.5243E+02,+7.7229E+02]
  -2.60197386356484E+02 .*             N9_cv.^3  + ... % SE: 5.0820E+01, PE:  19.53%, 95% CI: [-3.6184E+02,-1.5856E+02]
  +2.54365678012454E+02 .*                    1 ;      % SE: 6.2094E+01, PE:  24.41%, 95% CI: [+1.3018E+02,+3.7855E+02]

% Flift Model
RSQ(7)=91.662; PSE(7)=7.7786E+05; PRESS(7)=5.6683E+08; NTERMS(7)=21;
Flift = ...
  +2.96410387266717E+03 .*                 u_cv  + ... % SE: 1.3368E+02, PE:   4.51%, 95% CI: [+2.6967E+03,+3.2315E+03]
  +3.20259946803112E+03 .*                 w_cv  + ... % SE: 1.2986E+02, PE:   4.05%, 95% CI: [+2.9429E+03,+3.4623E+03]
  +5.32973437754410E+02 .*                N1_cv  + ... % SE: 3.6449E+01, PE:   6.84%, 95% CI: [+4.6008E+02,+6.0587E+02]
  +6.24592523906435E+02 .*                N2_cv  + ... % SE: 3.6607E+01, PE:   5.86%, 95% CI: [+5.5138E+02,+6.9781E+02]
  +4.75808574314941E+02 .*                N3_cv  + ... % SE: 3.6534E+01, PE:   7.68%, 95% CI: [+4.0274E+02,+5.4888E+02]
  +5.90681474048818E+02 .*                N4_cv  + ... % SE: 3.6614E+01, PE:   6.20%, 95% CI: [+5.1745E+02,+6.6391E+02]
  +4.80455217150644E+02 .*                N5_cv  + ... % SE: 3.6405E+01, PE:   7.58%, 95% CI: [+4.0765E+02,+5.5326E+02]
  +6.41220869487874E+02 .*                N6_cv  + ... % SE: 3.6490E+01, PE:   5.69%, 95% CI: [+5.6824E+02,+7.1420E+02]
  +5.71170005675471E+02 .*                N7_cv  + ... % SE: 3.6485E+01, PE:   6.39%, 95% CI: [+4.9820E+02,+6.4414E+02]
  +7.02498626298932E+02 .*                N8_cv  + ... % SE: 3.6570E+01, PE:   5.21%, 95% CI: [+6.2936E+02,+7.7564E+02]
  -7.42493992294296E+02 .*         u_cv .* w_cv  + ... % SE: 1.0613E+02, PE:  14.29%, 95% CI: [-9.5476E+02,-5.3023E+02]
  -3.02077565353370E+03 .*              w_cv.^2  + ... % SE: 8.4887E+01, PE:   2.81%, 95% CI: [-3.1906E+03,-2.8510E+03]
  +4.34062381272085E+02 .*             N1_cv.^2  + ... % SE: 8.6338E+01, PE:  19.89%, 95% CI: [+2.6139E+02,+6.0674E+02]
  +7.34016090067456E+02 .*        u_cv .* N2_cv  + ... % SE: 1.0551E+02, PE:  14.37%, 95% CI: [+5.2299E+02,+9.4504E+02]
  +3.62938570297443E+02 .*             N4_cv.^2  + ... % SE: 8.5446E+01, PE:  23.54%, 95% CI: [+1.9205E+02,+5.3383E+02]
  +4.76130869757898E+02 .*        u_cv .* N6_cv  + ... % SE: 1.0409E+02, PE:  21.86%, 95% CI: [+2.6796E+02,+6.8430E+02]
  +3.72151346143776E+02 .*             N6_cv.^2  + ... % SE: 8.6487E+01, PE:  23.24%, 95% CI: [+1.9918E+02,+5.4512E+02]
  +1.37071537709482E+03 .*        w_cv .* N9_cv  + ... % SE: 4.2361E+01, PE:   3.09%, 95% CI: [+1.2860E+03,+1.4554E+03]
  -2.55638451365229E+03 .*              u_cv.^3  + ... % SE: 1.5482E+02, PE:   6.06%, 95% CI: [-2.8660E+03,-2.2468E+03]
  -1.08109139324683E+03 .*              w_cv.^3  + ... % SE: 1.4060E+02, PE:  13.01%, 95% CI: [-1.3623E+03,-7.9989E+02]
  +7.37259031847298E+03 .*                    1 ;      % SE: 4.7409E+01, PE:   0.64%, 95% CI: [+7.2778E+03,+7.4674E+03]

% Fdrag Model
RSQ(8)=98.249; PSE(8)=9.6339E+05; PRESS(8)=3.7312E+08; NTERMS(8)=21;
Fdrag = ...
  -1.14437531618635E+03 .*                 u_cv  + ... % SE: 1.0953E+02, PE:   9.57%, 95% CI: [-1.3634E+03,-9.2532E+02]
  +8.54818285368895E+03 .*                 w_cv  + ... % SE: 1.0613E+02, PE:   1.24%, 95% CI: [+8.3359E+03,+8.7605E+03]
  -8.10510431811678E+02 .*                N9_cv  + ... % SE: 2.9810E+01, PE:   3.68%, 95% CI: [-8.7013E+02,-7.5089E+02]
  -3.90205932493306E+02 .*              u_cv.^2  + ... % SE: 7.5222E+01, PE:  19.28%, 95% CI: [-5.4065E+02,-2.3976E+02]
  +6.59673765536989E+02 .*              v_cv.^2  + ... % SE: 6.9357E+01, PE:  10.51%, 95% CI: [+5.2096E+02,+7.9839E+02]
  -1.75840799219141E+03 .*         u_cv .* w_cv  + ... % SE: 8.6661E+01, PE:   4.93%, 95% CI: [-1.9317E+03,-1.5851E+03]
  +6.37241979720507E+02 .*              w_cv.^2  + ... % SE: 6.7588E+01, PE:  10.61%, 95% CI: [+5.0207E+02,+7.7242E+02]
  +5.71901089056683E+02 .*        w_cv .* N1_cv  + ... % SE: 3.4718E+01, PE:   6.07%, 95% CI: [+5.0247E+02,+6.4134E+02]
  +6.52661440274926E+02 .*        w_cv .* N2_cv  + ... % SE: 3.4926E+01, PE:   5.35%, 95% CI: [+5.8281E+02,+7.2251E+02]
  +1.45735713713179E+02 .*        v_cv .* N3_cv  + ... % SE: 3.4316E+01, PE:  23.55%, 95% CI: [+7.7103E+01,+2.1437E+02]
  +5.46592375026866E+02 .*        w_cv .* N3_cv  + ... % SE: 3.4852E+01, PE:   6.38%, 95% CI: [+4.7689E+02,+6.1630E+02]
  +6.97890124563020E+02 .*        w_cv .* N4_cv  + ... % SE: 3.4939E+01, PE:   5.01%, 95% CI: [+6.2801E+02,+7.6777E+02]
  -2.02283278639835E+02 .*        v_cv .* N5_cv  + ... % SE: 3.4188E+01, PE:  16.90%, 95% CI: [-2.7066E+02,-1.3391E+02]
  +4.61033087911854E+02 .*        w_cv .* N5_cv  + ... % SE: 3.4801E+01, PE:   7.55%, 95% CI: [+3.9143E+02,+5.3064E+02]
  +6.13165059419862E+02 .*        w_cv .* N6_cv  + ... % SE: 3.4848E+01, PE:   5.68%, 95% CI: [+5.4347E+02,+6.8286E+02]
  +5.71018835099512E+02 .*        w_cv .* N7_cv  + ... % SE: 3.4749E+01, PE:   6.09%, 95% CI: [+5.0152E+02,+6.4052E+02]
  +7.24018046731080E+02 .*        w_cv .* N8_cv  + ... % SE: 3.4941E+01, PE:   4.83%, 95% CI: [+6.5414E+02,+7.9390E+02]
  -4.65925430461846E+02 .*        u_cv .* N9_cv  + ... % SE: 8.5691E+01, PE:  18.39%, 95% CI: [-6.3731E+02,-2.9454E+02]
  +1.18086693618285E+03 .*              u_cv.^3  + ... % SE: 1.2691E+02, PE:  10.75%, 95% CI: [+9.2706E+02,+1.4347E+03]
  -2.64294337145675E+03 .*              w_cv.^3  + ... % SE: 1.1493E+02, PE:   4.35%, 95% CI: [-2.8728E+03,-2.4131E+03]
  -1.68189820655875E+03 .*                    1 ;      % SE: 6.2798E+01, PE:   3.73%, 95% CI: [-1.8075E+03,-1.5563E+03]

% Modeling Metrics
Metrics.RSQ=RSQ;
Metrics.PSE=PSE;
Metrics.PRESS=PRESS;
Metrics.NTERMS=NTERMS;

C = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];

return