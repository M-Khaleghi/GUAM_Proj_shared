function [C,Metrics]=LpC_Hover_R12t15_PCp2FI_MOF_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8)
% LpC_Hover_R12t15_PCp2FI_MOF_cv - Aerodynamic model for the LpC Hover
%
% DESCRIPTION: 
%   This script contains the aerodynamic model for LpC Hover. The script was
%   automatically generated using "GenModelCV.m" on 16-Mar-2021 14:08:05
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
RSQ(1)=93.352; PSE(1)=3011.8; PRESS(1)=2.1389E+06; NTERMS(1)=63;
Faxial = ...
  +3.57163464004787E+02 .*                    1  + ... % SE: 3.9277E+00, PE:   1.10%, 95% CI: [+3.4931E+02,+3.6502E+02]
  +1.72250460095601E+02 .*                 u_cv  + ... % SE: 2.5395E+00, PE:   1.47%, 95% CI: [+1.6717E+02,+1.7733E+02]
  -1.00664019975519E+02 .*                 w_cv  + ... % SE: 2.5269E+00, PE:   2.51%, 95% CI: [-1.0572E+02,-9.5610E+01]
  +4.25375954503909E+01 .*                N2_cv  + ... % SE: 2.5501E+00, PE:   5.99%, 95% CI: [+3.7437E+01,+4.7638E+01]
  +3.96475406620995E+01 .*                N8_cv  + ... % SE: 2.5476E+00, PE:   6.43%, 95% CI: [+3.4552E+01,+4.4743E+01]
  +3.73740150587072E+01 .*                N1_cv  + ... % SE: 2.5324E+00, PE:   6.78%, 95% CI: [+3.2309E+01,+4.2439E+01]
  +3.57022119428262E+01 .*                N5_cv  + ... % SE: 2.5659E+00, PE:   7.19%, 95% CI: [+3.0570E+01,+4.0834E+01]
  +3.57652713241289E+01 .*                N3_cv  + ... % SE: 2.5331E+00, PE:   7.08%, 95% CI: [+3.0699E+01,+4.0832E+01]
  +1.95799086641126E+01 .*             N7_cv.^3  + ... % SE: 9.8046E+00, PE:  50.08%, 95% CI: [-2.9370E-02,+3.9189E+01]
  +2.75950819312312E+01 .*                N4_cv  + ... % SE: 2.5542E+00, PE:   9.26%, 95% CI: [+2.2487E+01,+3.2704E+01]
  +2.34179298534522E+01 .*            RUD_cv.^2  + ... % SE: 6.4284E+00, PE:  27.45%, 95% CI: [+1.0561E+01,+3.6275E+01]
  +2.64344029570394E+01 .*             N6_cv.^3  + ... % SE: 2.7537E+00, PE:  10.42%, 95% CI: [+2.0927E+01,+3.1942E+01]
  -2.29024305272203E+01 .*        v_cv .* N4_cv  + ... % SE: 2.9464E+00, PE:  12.87%, 95% CI: [-2.8795E+01,-1.7010E+01]
  -2.14828021886866E+01 .*         u_cv .* w_cv  + ... % SE: 2.9254E+00, PE:  13.62%, 95% CI: [-2.7334E+01,-1.5632E+01]
  +2.20298889600397E+01 .*        v_cv .* N6_cv  + ... % SE: 2.9505E+00, PE:  13.39%, 95% CI: [+1.6129E+01,+2.7931E+01]
  +2.36570239325772E+01 .*        u_cv .* N3_cv  + ... % SE: 2.9477E+00, PE:  12.46%, 95% CI: [+1.7762E+01,+2.9553E+01]
  -2.00011518104372E+01 .*        w_cv .* N4_cv  + ... % SE: 2.9522E+00, PE:  14.76%, 95% CI: [-2.5906E+01,-1.4097E+01]
  +2.21533223804505E+01 .*        u_cv .* N5_cv  + ... % SE: 2.9739E+00, PE:  13.42%, 95% CI: [+1.6205E+01,+2.8101E+01]
  -2.07006074481989E+01 .*        w_cv .* N6_cv  + ... % SE: 2.9366E+00, PE:  14.19%, 95% CI: [-2.6574E+01,-1.4827E+01]
  +2.16273209569521E+01 .*              u_cv.^2  + ... % SE: 6.4214E+00, PE:  29.69%, 95% CI: [+8.7846E+00,+3.4470E+01]
  -4.88761204323910E+01 .*              w_cv.^2  + ... % SE: 6.4264E+00, PE:  13.15%, 95% CI: [-6.1729E+01,-3.6023E+01]
  +1.53201492652462E+01 .*        w_cv .* N3_cv  + ... % SE: 2.9152E+00, PE:  19.03%, 95% CI: [+9.4897E+00,+2.1151E+01]
  +1.52988974723337E+01 .*        v_cv .* N3_cv  + ... % SE: 2.9337E+00, PE:  19.18%, 95% CI: [+9.4315E+00,+2.1166E+01]
  -1.35673489569457E+01 .*        v_cv .* N1_cv  + ... % SE: 2.9396E+00, PE:  21.67%, 95% CI: [-1.9447E+01,-7.6881E+00]
  +1.83861054903363E+01 .*             N4_cv.^2  + ... % SE: 6.3429E+00, PE:  34.50%, 95% CI: [+5.7003E+00,+3.1072E+01]
  -1.21759114476115E+01 .*        v_cv .* N5_cv  + ... % SE: 2.9683E+00, PE:  24.38%, 95% CI: [-1.8112E+01,-6.2394E+00]
  -1.02005324451959E+01 .*        w_cv .* N7_cv  + ... % SE: 2.9388E+00, PE:  28.81%, 95% CI: [-1.6078E+01,-4.3229E+00]
  +1.33610162541634E+01 .*        w_cv .* N5_cv  + ... % SE: 2.9551E+00, PE:  22.12%, 95% CI: [+7.4508E+00,+1.9271E+01]
  +1.04711092983707E+01 .*       N3_cv .* N4_cv  + ... % SE: 2.9540E+00, PE:  28.21%, 95% CI: [+4.5631E+00,+1.6379E+01]
  +1.22048537597243E+01 .*        v_cv .* N7_cv  + ... % SE: 2.9459E+00, PE:  24.14%, 95% CI: [+6.3130E+00,+1.8097E+01]
  +2.41786689217314E+01 .*                LA_cv  + ... % SE: 8.9098E+00, PE:  36.85%, 95% CI: [+6.3591E+00,+4.1998E+01]
  -9.59318910204651E+00 .*       N4_cv .* N6_cv  + ... % SE: 2.9834E+00, PE:  31.10%, 95% CI: [-1.5560E+01,-3.6264E+00]
  +8.25938344606145E+00 .*                RA_cv  + ... % SE: 2.5469E+00, PE:  30.84%, 95% CI: [+3.1656E+00,+1.3353E+01]
  +8.99450741810891E+00 .*        u_cv .* N1_cv  + ... % SE: 2.9484E+00, PE:  32.78%, 95% CI: [+3.0977E+00,+1.4891E+01]
  +1.23344577377469E+01 .*             RA_cv.^2  + ... % SE: 6.4288E+00, PE:  52.12%, 95% CI: [-5.2319E-01,+2.5192E+01]
  +9.84934788799272E+00 .*        u_cv .* N8_cv  + ... % SE: 2.9448E+00, PE:  29.90%, 95% CI: [+3.9597E+00,+1.5739E+01]
  -9.05332116284450E+00 .*       N2_cv .* N6_cv  + ... % SE: 2.9765E+00, PE:  32.88%, 95% CI: [-1.5006E+01,-3.1002E+00]
  -9.46284713091043E+00 .*       LA_cv .* RA_cv  + ... % SE: 2.9507E+00, PE:  31.18%, 95% CI: [-1.5364E+01,-3.5614E+00]
  +8.53390017237989E+00 .*       LA_cv .* N4_cv  + ... % SE: 2.9641E+00, PE:  34.73%, 95% CI: [+2.6057E+00,+1.4462E+01]
  +8.84294253416807E+00 .*        u_cv .* N7_cv  + ... % SE: 2.9456E+00, PE:  33.31%, 95% CI: [+2.9517E+00,+1.4734E+01]
  -8.48698046382440E+00 .*       LA_cv .* LE_cv  + ... % SE: 2.9358E+00, PE:  34.59%, 95% CI: [-1.4359E+01,-2.6154E+00]
  +9.02843623774820E+00 .*       N2_cv .* N3_cv  + ... % SE: 2.9687E+00, PE:  32.88%, 95% CI: [+3.0911E+00,+1.4966E+01]
  -7.88123817733996E+00 .*       LE_cv .* N1_cv  + ... % SE: 2.9305E+00, PE:  37.18%, 95% CI: [-1.3742E+01,-2.0202E+00]
  +9.11892047738407E+00 .*       N7_cv .* N8_cv  + ... % SE: 2.9628E+00, PE:  32.49%, 95% CI: [+3.1932E+00,+1.5045E+01]
  +8.64064400542505E+00 .*       N2_cv .* N5_cv  + ... % SE: 2.9876E+00, PE:  34.58%, 95% CI: [+2.6654E+00,+1.4616E+01]
  +8.63490294749971E+00 .*       N5_cv .* N8_cv  + ... % SE: 3.0098E+00, PE:  34.86%, 95% CI: [+2.6153E+00,+1.4654E+01]
  +1.64480232382143E+01 .*             N2_cv.^2  + ... % SE: 6.3210E+00, PE:  38.43%, 95% CI: [+3.8060E+00,+2.9090E+01]
  -7.44932455058234E+00 .*       N1_cv .* N4_cv  + ... % SE: 2.9595E+00, PE:  39.73%, 95% CI: [-1.3368E+01,-1.5304E+00]
  -7.37610422064563E+00 .*       N6_cv .* N7_cv  + ... % SE: 2.9619E+00, PE:  40.15%, 95% CI: [-1.3300E+01,-1.4524E+00]
  +7.26047525584000E+00 .*      LA_cv .* RUD_cv  + ... % SE: 2.9637E+00, PE:  40.82%, 95% CI: [+1.3331E+00,+1.3188E+01]
  +7.32561717794392E+00 .*       N1_cv .* N5_cv  + ... % SE: 2.9683E+00, PE:  40.52%, 95% CI: [+1.3891E+00,+1.3262E+01]
  +1.44480899008964E+01 .*             N6_cv.^2  + ... % SE: 6.3588E+00, PE:  44.01%, 95% CI: [+1.7305E+00,+2.7166E+01]
  -5.85835270229044E+00 .*      RE_cv .* RUD_cv  + ... % SE: 2.9906E+00, PE:  51.05%, 95% CI: [-1.1839E+01,+1.2275E-01]
  -5.97909202749490E+00 .*       N1_cv .* N8_cv  + ... % SE: 2.9594E+00, PE:  49.50%, 95% CI: [-1.1898E+01,-6.0268E-02]
  -6.52168652154876E+00 .*       RE_cv .* N4_cv  + ... % SE: 2.9978E+00, PE:  45.97%, 95% CI: [-1.2517E+01,-5.2607E-01]
  +6.65052019975806E+00 .*       RA_cv .* N5_cv  + ... % SE: 2.9772E+00, PE:  44.77%, 95% CI: [+6.9620E-01,+1.2605E+01]
  +5.88582880958459E+00 .*        u_cv .* LA_cv  + ... % SE: 2.9323E+00, PE:  49.82%, 95% CI: [+2.1186E-02,+1.1750E+01]
  -5.95287683765903E+00 .*        u_cv .* LE_cv  + ... % SE: 2.9126E+00, PE:  48.93%, 95% CI: [-1.1778E+01,-1.2774E-01]
  -5.91194020669595E+00 .*      RUD_cv .* N8_cv  + ... % SE: 2.9603E+00, PE:  50.07%, 95% CI: [-1.1833E+01,+8.6234E-03]
  +1.66567637352862E+01 .*                N7_cv  + ... % SE: 9.0600E+00, PE:  54.39%, 95% CI: [-1.4631E+00,+3.4777E+01]
  -5.33566151429588E+00 .*       RA_cv .* N3_cv  + ... % SE: 2.9483E+00, PE:  55.26%, 95% CI: [-1.1232E+01,+5.6085E-01]
  -5.21309229002035E+00 .*       LE_cv .* N7_cv  + ... % SE: 2.9568E+00, PE:  56.72%, 95% CI: [-1.1127E+01,+7.0046E-01]
  -1.60053109200444E+01 .*             LA_cv.^3 ;      % SE: 9.6641E+00, PE:  60.38%, 95% CI: [-3.5333E+01,+3.3229E+00]

% Fside Model
RSQ(2)=96.242; PSE(2)=4956.9; PRESS(2)=3.1341E+06; NTERMS(2)=42;
Fside = ...
  -2.18405114064757E+02 .*                N3_cv  + ... % SE: 1.1162E+01, PE:   5.11%, 95% CI: [-2.4073E+02,-1.9608E+02]
  +1.90373503721396E+02 .*                N5_cv  + ... % SE: 3.1735E+00, PE:   1.67%, 95% CI: [+1.8403E+02,+1.9672E+02]
  -1.82699631885286E+02 .*                 v_cv  + ... % SE: 3.1392E+00, PE:   1.72%, 95% CI: [-1.8898E+02,-1.7642E+02]
  -1.60597565229015E+02 .*                N4_cv  + ... % SE: 3.1617E+00, PE:   1.97%, 95% CI: [-1.6692E+02,-1.5427E+02]
  +1.53675038697068E+02 .*                N6_cv  + ... % SE: 3.1485E+00, PE:   2.05%, 95% CI: [+1.4738E+02,+1.5997E+02]
  -4.97229913699655E+01 .*               RUD_cv  + ... % SE: 3.1587E+00, PE:   6.35%, 95% CI: [-5.6040E+01,-4.3406E+01]
  +3.62437584415461E+01 .*         u_cv .* v_cv  + ... % SE: 3.6122E+00, PE:   9.97%, 95% CI: [+2.9019E+01,+4.3468E+01]
  -2.86820202211694E+01 .*       u_cv .* RUD_cv  + ... % SE: 3.6313E+00, PE:  12.66%, 95% CI: [-3.5945E+01,-2.1419E+01]
  -2.39686331072052E+01 .*         v_cv .* w_cv  + ... % SE: 3.6142E+00, PE:  15.08%, 95% CI: [-3.1197E+01,-1.6740E+01]
  -2.31832314141299E+01 .*        w_cv .* N5_cv  + ... % SE: 3.6355E+00, PE:  15.68%, 95% CI: [-3.0454E+01,-1.5912E+01]
  +2.35506573968857E+01 .*        w_cv .* N3_cv  + ... % SE: 3.6038E+00, PE:  15.30%, 95% CI: [+1.6343E+01,+3.0758E+01]
  +1.88771114397595E+01 .*        u_cv .* N6_cv  + ... % SE: 3.6400E+00, PE:  19.28%, 95% CI: [+1.1597E+01,+2.6157E+01]
  -6.47663900342239E+01 .*             N3_cv.^2  + ... % SE: 7.3606E+00, PE:  11.36%, 95% CI: [-7.9488E+01,-5.0045E+01]
  +6.63752527620345E+01 .*             N5_cv.^2  + ... % SE: 7.1968E+00, PE:  10.84%, 95% CI: [+5.1982E+01,+8.0769E+01]
  -1.57471080977158E+01 .*        w_cv .* N4_cv  + ... % SE: 3.6419E+00, PE:  23.13%, 95% CI: [-2.3031E+01,-8.4632E+00]
  -1.42693433960184E+01 .*        u_cv .* N4_cv  + ... % SE: 3.6521E+00, PE:  25.59%, 95% CI: [-2.1574E+01,-6.9651E+00]
  +1.51770140649063E+01 .*        w_cv .* N6_cv  + ... % SE: 3.6313E+00, PE:  23.93%, 95% CI: [+7.9144E+00,+2.2440E+01]
  -1.22920245580326E+01 .*        v_cv .* N8_cv  + ... % SE: 3.6597E+00, PE:  29.77%, 95% CI: [-1.9611E+01,-4.9726E+00]
  +1.33216354100371E+01 .*       N3_cv .* N4_cv  + ... % SE: 3.6347E+00, PE:  27.28%, 95% CI: [+6.0523E+00,+2.0591E+01]
  -1.43776663516998E+01 .*        v_cv .* N2_cv  + ... % SE: 3.6303E+00, PE:  25.25%, 95% CI: [-2.1638E+01,-7.1171E+00]
  +1.25157797727103E+01 .*       w_cv .* RUD_cv  + ... % SE: 3.6356E+00, PE:  29.05%, 95% CI: [+5.2445E+00,+1.9787E+01]
  +1.15917343266445E+01 .*       LA_cv .* N7_cv  + ... % SE: 3.6444E+00, PE:  31.44%, 95% CI: [+4.3029E+00,+1.8881E+01]
  -2.71176390251187E+01 .*             N4_cv.^2  + ... % SE: 7.2627E+00, PE:  26.78%, 95% CI: [-4.1643E+01,-1.2592E+01]
  -1.06241836003055E+01 .*       N3_cv .* N7_cv  + ... % SE: 3.6305E+00, PE:  34.17%, 95% CI: [-1.7885E+01,-3.3633E+00]
  -1.03957214039422E+01 .*        v_cv .* N7_cv  + ... % SE: 3.6338E+00, PE:  34.95%, 95% CI: [-1.7663E+01,-3.1281E+00]
  +3.50642753396202E+01 .*             N3_cv.^3  + ... % SE: 1.2047E+01, PE:  34.36%, 95% CI: [+1.0971E+01,+5.9157E+01]
  +1.76855122680831E+01 .*             N6_cv.^2  + ... % SE: 7.3570E+00, PE:  41.60%, 95% CI: [+2.9716E+00,+3.2399E+01]
  -9.81445263245855E+00 .*       N6_cv .* N7_cv  + ... % SE: 3.6392E+00, PE:  37.08%, 95% CI: [-1.7093E+01,-2.5361E+00]
  -9.65941072261516E+00 .*        v_cv .* N1_cv  + ... % SE: 3.6063E+00, PE:  37.34%, 95% CI: [-1.6872E+01,-2.4467E+00]
  -1.00887073930092E+01 .*       N5_cv .* N6_cv  + ... % SE: 3.6788E+00, PE:  36.46%, 95% CI: [-1.7446E+01,-2.7312E+00]
  +1.04243812092793E+01 .*        w_cv .* N7_cv  + ... % SE: 3.6287E+00, PE:  34.81%, 95% CI: [+3.1670E+00,+1.7682E+01]
  -9.95506781567286E+00 .*             N2_cv.^3  + ... % SE: 3.4086E+00, PE:  34.24%, 95% CI: [-1.6772E+01,-3.1379E+00]
  -9.74117251108992E+00 .*        v_cv .* N4_cv  + ... % SE: 3.6565E+00, PE:  37.54%, 95% CI: [-1.7054E+01,-2.4281E+00]
  -8.59119399621313E+00 .*                N1_cv  + ... % SE: 3.1313E+00, PE:  36.45%, 95% CI: [-1.4854E+01,-2.3286E+00]
  +9.65216358795503E+00 .*      RUD_cv .* N1_cv  + ... % SE: 3.6495E+00, PE:  37.81%, 95% CI: [+2.3532E+00,+1.6951E+01]
  -1.01312347903212E+01 .*        w_cv .* N1_cv  + ... % SE: 3.6253E+00, PE:  35.78%, 95% CI: [-1.7382E+01,-2.8807E+00]
  -9.35645048969002E+00 .*      LA_cv .* RUD_cv  + ... % SE: 3.6454E+00, PE:  38.96%, 95% CI: [-1.6647E+01,-2.0657E+00]
  -9.15187091828390E+00 .*       RA_cv .* N5_cv  + ... % SE: 3.6635E+00, PE:  40.03%, 95% CI: [-1.6479E+01,-1.8249E+00]
  +9.28965527837581E+00 .*       N7_cv .* N8_cv  + ... % SE: 3.6647E+00, PE:  39.45%, 95% CI: [+1.9604E+00,+1.6619E+01]
  +9.17788778974122E+00 .*       RE_cv .* N5_cv  + ... % SE: 3.7207E+00, PE:  40.54%, 95% CI: [+1.7364E+00,+1.6619E+01]
  -9.05337873458066E+00 .*       N2_cv .* N4_cv  + ... % SE: 3.6732E+00, PE:  40.57%, 95% CI: [-1.6400E+01,-1.7069E+00]
  -8.64437236905079E+00 .*        w_cv .* N2_cv ;      % SE: 3.6158E+00, PE:  41.83%, 95% CI: [-1.5876E+01,-1.4128E+00]

% Fnormal Model
RSQ(3)=98.958; PSE(3)=99486; PRESS(3)=4.4171E+07; NTERMS(3)=29;
Fnormal = ...
  +8.87282693760624E+03 .*                    1  + ... % SE: 1.8883E+01, PE:   0.21%, 95% CI: [+8.8351E+03,+8.9106E+03]
  +1.20135740290698E+03 .*                 w_cv  + ... % SE: 1.1933E+01, PE:   0.99%, 95% CI: [+1.1775E+03,+1.2252E+03]
  +1.16896293316574E+03 .*                N4_cv  + ... % SE: 1.2107E+01, PE:   1.04%, 95% CI: [+1.1447E+03,+1.1932E+03]
  +1.18708606355321E+03 .*                N6_cv  + ... % SE: 1.2059E+01, PE:   1.02%, 95% CI: [+1.1630E+03,+1.2112E+03]
  +1.11247435932502E+03 .*                N2_cv  + ... % SE: 1.2065E+01, PE:   1.08%, 95% CI: [+1.0883E+03,+1.1366E+03]
  +1.10659288539594E+03 .*                N8_cv  + ... % SE: 1.2052E+01, PE:   1.09%, 95% CI: [+1.0825E+03,+1.1307E+03]
  +7.62254065533589E+02 .*                N7_cv  + ... % SE: 1.2032E+01, PE:   1.58%, 95% CI: [+7.3819E+02,+7.8632E+02]
  +7.53648908830186E+02 .*                N1_cv  + ... % SE: 1.1988E+01, PE:   1.59%, 95% CI: [+7.2967E+02,+7.7763E+02]
  +6.68845079411481E+02 .*                N3_cv  + ... % SE: 1.1988E+01, PE:   1.79%, 95% CI: [+6.4487E+02,+6.9282E+02]
  +6.72063052336578E+02 .*                N5_cv  + ... % SE: 1.2158E+01, PE:   1.81%, 95% CI: [+6.4775E+02,+6.9638E+02]
  +4.48805778097462E+02 .*                 u_cv  + ... % SE: 1.1967E+01, PE:   2.67%, 95% CI: [+4.2487E+02,+4.7274E+02]
  +1.95403926659180E+02 .*              w_cv.^2  + ... % SE: 3.0936E+01, PE:  15.83%, 95% CI: [+1.3353E+02,+2.5728E+02]
  +3.08955547705547E+02 .*         u_cv .* w_cv  + ... % SE: 1.3801E+01, PE:   4.47%, 95% CI: [+2.8135E+02,+3.3656E+02]
  +1.69301095225899E+02 .*             N6_cv.^2  + ... % SE: 3.0583E+01, PE:  18.06%, 95% CI: [+1.0814E+02,+2.3047E+02]
  +1.74149998049239E+02 .*             N8_cv.^2  + ... % SE: 3.0546E+01, PE:  17.54%, 95% CI: [+1.1306E+02,+2.3524E+02]
  +1.26957381814228E+02 .*                LA_cv  + ... % SE: 1.2032E+01, PE:   9.48%, 95% CI: [+1.0289E+02,+1.5102E+02]
  +1.21174980996445E+02 .*                RA_cv  + ... % SE: 1.2002E+01, PE:   9.90%, 95% CI: [+9.7172E+01,+1.4518E+02]
  +1.64011232937325E+02 .*             N7_cv.^2  + ... % SE: 3.0596E+01, PE:  18.65%, 95% CI: [+1.0282E+02,+2.2520E+02]
  +1.21588457591342E+02 .*        w_cv .* N6_cv  + ... % SE: 1.3845E+01, PE:  11.39%, 95% CI: [+9.3899E+01,+1.4928E+02]
  +1.09416761199758E+02 .*        w_cv .* N4_cv  + ... % SE: 1.3938E+01, PE:  12.74%, 95% CI: [+8.1540E+01,+1.3729E+02]
  +1.03362934942525E+02 .*        u_cv .* N6_cv  + ... % SE: 1.3881E+01, PE:  13.43%, 95% CI: [+7.5600E+01,+1.3113E+02]
  +1.82776821391500E+02 .*             N3_cv.^2  + ... % SE: 3.0636E+01, PE:  16.76%, 95% CI: [+1.2151E+02,+2.4405E+02]
  +1.02634053511110E+02 .*        u_cv .* N4_cv  + ... % SE: 1.3903E+01, PE:  13.55%, 95% CI: [+7.4829E+01,+1.3044E+02]
  +8.97009057131097E+01 .*        w_cv .* N2_cv  + ... % SE: 1.3850E+01, PE:  15.44%, 95% CI: [+6.2000E+01,+1.1740E+02]
  +1.63155383357721E+02 .*             N2_cv.^2  + ... % SE: 3.0390E+01, PE:  18.63%, 95% CI: [+1.0238E+02,+2.2394E+02]
  +8.16436469158500E+01 .*        w_cv .* N8_cv  + ... % SE: 1.3880E+01, PE:  17.00%, 95% CI: [+5.3885E+01,+1.0940E+02]
  +1.34783257417982E+02 .*             N4_cv.^2  + ... % SE: 3.0341E+01, PE:  22.51%, 95% CI: [+7.4101E+01,+1.9547E+02]
  -1.43355327467195E+02 .*              v_cv.^2  + ... % SE: 3.1086E+01, PE:  21.68%, 95% CI: [-2.0553E+02,-8.1183E+01]
  +1.42837315870936E+02 .*             N1_cv.^2 ;      % SE: 3.0827E+01, PE:  21.58%, 95% CI: [+8.1183E+01,+2.0449E+02]

% Mroll Model
RSQ(4)=99.251; PSE(4)=1.0688E+07; PRESS(4)=4.4809E+09; NTERMS(4)=22;
Mroll = ...
  -1.98643036611109E+04 .*                N8_cv  + ... % SE: 1.2152E+02, PE:   0.61%, 95% CI: [-2.0107E+04,-1.9621E+04]
  +1.96163531306679E+04 .*                N2_cv  + ... % SE: 1.2174E+02, PE:   0.62%, 95% CI: [+1.9373E+04,+1.9860E+04]
  +1.37623009913244E+04 .*                N1_cv  + ... % SE: 1.2097E+02, PE:   0.88%, 95% CI: [+1.3520E+04,+1.4004E+04]
  -1.37343187734553E+04 .*                N7_cv  + ... % SE: 1.2132E+02, PE:   0.88%, 95% CI: [-1.3977E+04,-1.3492E+04]
  -7.71145055842772E+03 .*                N6_cv  + ... % SE: 1.2163E+02, PE:   1.58%, 95% CI: [-7.9547E+03,-7.4682E+03]
  +7.88602408783047E+03 .*                N4_cv  + ... % SE: 1.2199E+02, PE:   1.55%, 95% CI: [+7.6421E+03,+8.1300E+03]
  +4.66241862363265E+03 .*                N3_cv  + ... % SE: 1.2089E+02, PE:   2.59%, 95% CI: [+4.4206E+03,+4.9042E+03]
  -4.71171463206559E+03 .*                N5_cv  + ... % SE: 1.2276E+02, PE:   2.61%, 95% CI: [-4.9572E+03,-4.4662E+03]
  +1.73632516970305E+03 .*                LA_cv  + ... % SE: 1.2119E+02, PE:   6.98%, 95% CI: [+1.4939E+03,+1.9787E+03]
  -1.71348299516345E+03 .*                RA_cv  + ... % SE: 1.2119E+02, PE:   7.07%, 95% CI: [-1.9559E+03,-1.4711E+03]
  -1.71399484896984E+03 .*         v_cv .* w_cv  + ... % SE: 1.3972E+02, PE:   8.15%, 95% CI: [-1.9934E+03,-1.4346E+03]
  -1.18529227636964E+03 .*                 v_cv  + ... % SE: 1.2108E+02, PE:  10.22%, 95% CI: [-1.4275E+03,-9.4313E+02]
  -1.03930763503581E+03 .*        w_cv .* N1_cv  + ... % SE: 1.3918E+02, PE:  13.39%, 95% CI: [-1.3177E+03,-7.6095E+02]
  +9.93536227491809E+02 .*        w_cv .* N7_cv  + ... % SE: 1.3944E+02, PE:  14.03%, 95% CI: [+7.1466E+02,+1.2724E+03]
  +9.37435233761027E+02 .*        u_cv .* LA_cv  + ... % SE: 1.3940E+02, PE:  14.87%, 95% CI: [+6.5863E+02,+1.2162E+03]
  -9.37632800744642E+02 .*        w_cv .* N8_cv  + ... % SE: 1.3977E+02, PE:  14.91%, 95% CI: [-1.2172E+03,-6.5810E+02]
  -8.32548403739681E+02 .*        u_cv .* RA_cv  + ... % SE: 1.3989E+02, PE:  16.80%, 95% CI: [-1.1123E+03,-5.5277E+02]
  +6.85471441461560E+02 .*        w_cv .* N2_cv  + ... % SE: 1.3965E+02, PE:  20.37%, 95% CI: [+4.0617E+02,+9.6477E+02]
  +3.52852549718517E+03 .*             N2_cv.^2  + ... % SE: 2.8124E+02, PE:   7.97%, 95% CI: [+2.9661E+03,+4.0910E+03]
  -3.42500957190401E+03 .*             N8_cv.^2  + ... % SE: 2.8270E+02, PE:   8.25%, 95% CI: [-3.9904E+03,-2.8596E+03]
  -2.96420172129674E+03 .*             N7_cv.^2  + ... % SE: 2.8385E+02, PE:   9.58%, 95% CI: [-3.5319E+03,-2.3965E+03]
  +2.89583980057180E+03 .*             N1_cv.^2 ;      % SE: 2.8473E+02, PE:   9.83%, 95% CI: [+2.3264E+03,+3.4653E+03]

% Mpitch Model
RSQ(5)=99.035; PSE(5)=1.0031E+06; PRESS(5)=4.1772E+08; NTERMS(5)=30;
Mpitch = ...
  +3.68386886679220E+03 .*                N3_cv  + ... % SE: 3.6772E+01, PE:   1.00%, 95% CI: [+3.6103E+03,+3.7574E+03]
  +3.68727436000804E+03 .*                N5_cv  + ... % SE: 3.7276E+01, PE:   1.01%, 95% CI: [+3.6127E+03,+3.7618E+03]
  +3.39497623262367E+03 .*                N7_cv  + ... % SE: 3.6994E+01, PE:   1.09%, 95% CI: [+3.3210E+03,+3.4690E+03]
  +3.38521729256396E+03 .*                N1_cv  + ... % SE: 3.6791E+01, PE:   1.09%, 95% CI: [+3.3116E+03,+3.4588E+03]
  -3.40470982416229E+03 .*                N2_cv  + ... % SE: 3.7016E+01, PE:   1.09%, 95% CI: [-3.4787E+03,-3.3307E+03]
  -3.37783527528972E+03 .*                N8_cv  + ... % SE: 3.7008E+01, PE:   1.10%, 95% CI: [-3.4519E+03,-3.3038E+03]
  +2.88225851003144E+03 .*                    1  + ... % SE: 5.7646E+01, PE:   2.00%, 95% CI: [+2.7670E+03,+2.9976E+03]
  -2.91755256344785E+03 .*                N6_cv  + ... % SE: 3.6962E+01, PE:   1.27%, 95% CI: [-2.9915E+03,-2.8436E+03]
  -2.88842859286586E+03 .*                N4_cv  + ... % SE: 3.7152E+01, PE:   1.29%, 95% CI: [-2.9627E+03,-2.8141E+03]
  -7.91123300976865E+02 .*             N2_cv.^2  + ... % SE: 9.2445E+01, PE:  11.69%, 95% CI: [-9.7601E+02,-6.0623E+02]
  -3.22153415303863E+02 .*                RE_cv  + ... % SE: 3.7297E+01, PE:  11.58%, 95% CI: [-3.9675E+02,-2.4756E+02]
  +3.42361431800527E+02 .*              u_cv.^3  + ... % SE: 3.9715E+01, PE:  11.60%, 95% CI: [+2.6293E+02,+4.2179E+02]
  +3.38487621821177E+02 .*        w_cv .* N3_cv  + ... % SE: 4.2368E+01, PE:  12.52%, 95% CI: [+2.5375E+02,+4.2322E+02]
  +3.53976944430379E+02 .*        w_cv .* N5_cv  + ... % SE: 4.2832E+01, PE:  12.10%, 95% CI: [+2.6831E+02,+4.3964E+02]
  -2.69833866318930E+02 .*                LE_cv  + ... % SE: 3.6800E+01, PE:  13.64%, 95% CI: [-3.4343E+02,-1.9623E+02]
  -2.79177103186949E+02 .*        v_cv .* N5_cv  + ... % SE: 4.3031E+01, PE:  15.41%, 95% CI: [-3.6524E+02,-1.9312E+02]
  +3.07163264246614E+02 .*        v_cv .* N3_cv  + ... % SE: 4.2503E+01, PE:  13.84%, 95% CI: [+2.2216E+02,+3.9217E+02]
  -2.48407992851214E+02 .*        v_cv .* N4_cv  + ... % SE: 4.2938E+01, PE:  17.29%, 95% CI: [-3.3428E+02,-1.6253E+02]
  +5.67576144296002E+02 .*             N5_cv.^2  + ... % SE: 9.2171E+01, PE:  16.24%, 95% CI: [+3.8323E+02,+7.5192E+02]
  -8.09069345162211E+02 .*             N8_cv.^2  + ... % SE: 9.2813E+01, PE:  11.47%, 95% CI: [-9.9469E+02,-6.2344E+02]
  +5.77660951037590E+02 .*             N1_cv.^2  + ... % SE: 9.4265E+01, PE:  16.32%, 95% CI: [+3.8913E+02,+7.6619E+02]
  -2.60829015107695E+02 .*         u_cv .* w_cv  + ... % SE: 4.2329E+01, PE:  16.23%, 95% CI: [-3.4549E+02,-1.7617E+02]
  -6.18106266161914E+02 .*             N4_cv.^2  + ... % SE: 9.2068E+01, PE:  14.90%, 95% CI: [-8.0224E+02,-4.3397E+02]
  +5.13931233318378E+02 .*             N7_cv.^2  + ... % SE: 9.3329E+01, PE:  18.16%, 95% CI: [+3.2727E+02,+7.0059E+02]
  -2.19921662964601E+02 .*        u_cv .* LE_cv  + ... % SE: 4.2215E+01, PE:  19.20%, 95% CI: [-3.0435E+02,-1.3549E+02]
  -2.11585394940918E+02 .*        u_cv .* RE_cv  + ... % SE: 4.3029E+01, PE:  20.34%, 95% CI: [-2.9764E+02,-1.2553E+02]
  +1.64122010307063E+02 .*                 w_cv  + ... % SE: 3.6694E+01, PE:  22.36%, 95% CI: [+9.0734E+01,+2.3751E+02]
  +1.96485832816910E+02 .*        v_cv .* N6_cv  + ... % SE: 4.2655E+01, PE:  21.71%, 95% CI: [+1.1118E+02,+2.8180E+02]
  +4.65896302743280E+02 .*              w_cv.^2  + ... % SE: 9.4281E+01, PE:  20.24%, 95% CI: [+2.7734E+02,+6.5446E+02]
  -4.55117130910195E+02 .*             N6_cv.^2 ;      % SE: 9.2999E+01, PE:  20.43%, 95% CI: [-6.4112E+02,-2.6912E+02]

% Myaw Model
RSQ(6)=97.25; PSE(6)=4.9754E+05; PRESS(6)=2.8536E+08; NTERMS(6)=42;
Myaw = ...
  +1.95592094975335E+03 .*                N4_cv  + ... % SE: 3.0284E+01, PE:   1.55%, 95% CI: [+1.8954E+03,+2.0165E+03]
  +1.88735141319313E+03 .*                N5_cv  + ... % SE: 3.0463E+01, PE:   1.61%, 95% CI: [+1.8264E+03,+1.9483E+03]
  -1.94771565856483E+03 .*                N3_cv  + ... % SE: 3.0014E+01, PE:   1.54%, 95% CI: [-2.0077E+03,-1.8877E+03]
  -1.90921103450052E+03 .*                N6_cv  + ... % SE: 3.0179E+01, PE:   1.58%, 95% CI: [-1.9696E+03,-1.8489E+03]
  +1.51367690203445E+03 .*                N8_cv  + ... % SE: 3.0187E+01, PE:   1.99%, 95% CI: [+1.4533E+03,+1.5741E+03]
  -1.47831847868129E+03 .*                N2_cv  + ... % SE: 3.0225E+01, PE:   2.04%, 95% CI: [-1.5388E+03,-1.4179E+03]
  +7.37842363093256E+02 .*               RUD_cv  + ... % SE: 3.0336E+01, PE:   4.11%, 95% CI: [+6.7717E+02,+7.9851E+02]
  +5.17066072655507E+02 .*       u_cv .* RUD_cv  + ... % SE: 3.4959E+01, PE:   6.76%, 95% CI: [+4.4715E+02,+5.8698E+02]
  +3.57373825310185E+02 .*        u_cv .* N4_cv  + ... % SE: 3.4914E+01, PE:   9.77%, 95% CI: [+2.8755E+02,+4.2720E+02]
  -3.46783129074005E+02 .*        v_cv .* N5_cv  + ... % SE: 3.5080E+01, PE:  10.12%, 95% CI: [-4.1694E+02,-2.7662E+02]
  -3.08518727863116E+02 .*        v_cv .* N3_cv  + ... % SE: 3.4725E+01, PE:  11.26%, 95% CI: [-3.7797E+02,-2.3907E+02]
  -2.71843929584420E+02 .*        u_cv .* N1_cv  + ... % SE: 3.4837E+01, PE:  12.81%, 95% CI: [-3.4152E+02,-2.0217E+02]
  -2.79672247960316E+02 .*        u_cv .* N6_cv  + ... % SE: 3.4751E+01, PE:  12.43%, 95% CI: [-3.4917E+02,-2.1017E+02]
  +2.76269914291945E+02 .*        u_cv .* N7_cv  + ... % SE: 3.4933E+01, PE:  12.64%, 95% CI: [+2.0640E+02,+3.4614E+02]
  +2.74693070287690E+02 .*        w_cv .* N4_cv  + ... % SE: 3.4939E+01, PE:  12.72%, 95% CI: [+2.0481E+02,+3.4457E+02]
  -2.14297188629469E+02 .*        w_cv .* N6_cv  + ... % SE: 3.4809E+01, PE:  16.24%, 95% CI: [-2.8391E+02,-1.4468E+02]
  +2.34069704353212E+02 .*        v_cv .* N4_cv  + ... % SE: 3.5060E+01, PE:  14.98%, 95% CI: [+1.6395E+02,+3.0419E+02]
  -2.05404927322277E+02 .*       w_cv .* RUD_cv  + ... % SE: 3.4852E+01, PE:  16.97%, 95% CI: [-2.7511E+02,-1.3570E+02]
  +2.19086528675574E+02 .*        v_cv .* N6_cv  + ... % SE: 3.4881E+01, PE:  15.92%, 95% CI: [+1.4933E+02,+2.8885E+02]
  +2.00342635541489E+02 .*         v_cv .* w_cv  + ... % SE: 3.4649E+01, PE:  17.29%, 95% CI: [+1.3105E+02,+2.6964E+02]
  -1.64596574001635E+02 .*                LA_cv  + ... % SE: 3.0117E+01, PE:  18.30%, 95% CI: [-2.2483E+02,-1.0436E+02]
  +1.89778303529204E+02 .*       N5_cv .* N6_cv  + ... % SE: 3.5396E+01, PE:  18.65%, 95% CI: [+1.1899E+02,+2.6057E+02]
  +6.17715872861260E+02 .*             N8_cv.^2  + ... % SE: 7.4429E+01, PE:  12.05%, 95% CI: [+4.6886E+02,+7.6657E+02]
  -5.90349547095267E+02 .*             N3_cv.^2  + ... % SE: 7.4722E+01, PE:  12.66%, 95% CI: [-7.3979E+02,-4.4091E+02]
  -5.72229896921965E+02 .*             N2_cv.^2  + ... % SE: 7.4253E+01, PE:  12.98%, 95% CI: [-7.2074E+02,-4.2372E+02]
  +4.77675140094754E+02 .*             N5_cv.^2  + ... % SE: 7.3744E+01, PE:  15.44%, 95% CI: [+3.3019E+02,+6.2516E+02]
  +1.47067943924790E+02 .*        u_cv .* N8_cv  + ... % SE: 3.4833E+01, PE:  23.69%, 95% CI: [+7.7401E+01,+2.1673E+02]
  +3.88731936139572E+02 .*             RA_cv.^2  + ... % SE: 7.4963E+01, PE:  19.28%, 95% CI: [+2.3881E+02,+5.3866E+02]
  -4.00275097643810E+02 .*             N6_cv.^2  + ... % SE: 7.4712E+01, PE:  18.67%, 95% CI: [-5.4970E+02,-2.5085E+02]
  +1.25980675363044E+02 .*             RA_cv.^3  + ... % SE: 3.2492E+01, PE:  25.79%, 95% CI: [+6.0997E+01,+1.9096E+02]
  -1.35903165394832E+02 .*       N2_cv .* N3_cv  + ... % SE: 3.4925E+01, PE:  25.70%, 95% CI: [-2.0575E+02,-6.6053E+01]
  -1.24967080253126E+02 .*        w_cv .* N1_cv  + ... % SE: 3.4661E+01, PE:  27.74%, 95% CI: [-1.9429E+02,-5.5645E+01]
  -1.35579320639314E+02 .*       N3_cv .* N4_cv  + ... % SE: 3.5043E+01, PE:  25.85%, 95% CI: [-2.0566E+02,-6.5494E+01]
  -1.21928515923829E+02 .*        u_cv .* RE_cv  + ... % SE: 3.5366E+01, PE:  29.01%, 95% CI: [-1.9266E+02,-5.1196E+01]
  -1.26077535656917E+02 .*       RA_cv .* N2_cv  + ... % SE: 3.4912E+01, PE:  27.69%, 95% CI: [-1.9590E+02,-5.6254E+01]
  +1.04629643044518E+02 .*                LE_cv  + ... % SE: 3.0063E+01, PE:  28.73%, 95% CI: [+4.4504E+01,+1.6476E+02]
  -1.07568412584019E+02 .*       N4_cv .* N7_cv  + ... % SE: 3.5366E+01, PE:  32.88%, 95% CI: [-1.7830E+02,-3.6837E+01]
  +1.31933477308622E+02 .*                    1  + ... % SE: 4.6359E+01, PE:  35.14%, 95% CI: [+3.9216E+01,+2.2465E+02]
  +1.03832765244587E+02 .*       N4_cv .* N8_cv  + ... % SE: 3.5261E+01, PE:  33.96%, 95% CI: [+3.3310E+01,+1.7436E+02]
  -9.96606588282909E+01 .*       RE_cv .* N5_cv  + ... % SE: 3.5629E+01, PE:  35.75%, 95% CI: [-1.7092E+02,-2.8402E+01]
  -9.42880248816848E+01 .*       N2_cv .* N6_cv  + ... % SE: 3.5158E+01, PE:  37.29%, 95% CI: [-1.6460E+02,-2.3971E+01]
  +9.31464079537483E+01 .*        v_cv .* N2_cv ;      % SE: 3.4873E+01, PE:  37.44%, 95% CI: [+2.3401E+01,+1.6289E+02]

% Flift Model
RSQ(7)=98.912; PSE(7)=96143; PRESS(7)=4.2759E+07; NTERMS(7)=30;
Flift = ...
  +8.90812225876903E+03 .*                    1  + ... % SE: 1.8328E+01, PE:   0.21%, 95% CI: [+8.8715E+03,+8.9448E+03]
  +1.12453454071561E+03 .*                N4_cv  + ... % SE: 1.1871E+01, PE:   1.06%, 95% CI: [+1.1008E+03,+1.1483E+03]
  +1.14390830014245E+03 .*                N6_cv  + ... % SE: 1.1816E+01, PE:   1.03%, 95% CI: [+1.1203E+03,+1.1675E+03]
  +1.06623146830463E+03 .*                N2_cv  + ... % SE: 1.1829E+01, PE:   1.11%, 95% CI: [+1.0426E+03,+1.0899E+03]
  +1.04961665980513E+03 .*                N8_cv  + ... % SE: 1.1828E+01, PE:   1.13%, 95% CI: [+1.0260E+03,+1.0733E+03]
  +1.03072030906540E+03 .*                 w_cv  + ... % SE: 1.1705E+01, PE:   1.14%, 95% CI: [+1.0073E+03,+1.0541E+03]
  +7.24534402667982E+02 .*                N7_cv  + ... % SE: 1.1806E+01, PE:   1.63%, 95% CI: [+7.0092E+02,+7.4815E+02]
  +7.24576330660154E+02 .*                 u_cv  + ... % SE: 1.1723E+01, PE:   1.62%, 95% CI: [+7.0113E+02,+7.4802E+02]
  +7.19481688624005E+02 .*                N1_cv  + ... % SE: 1.1758E+01, PE:   1.63%, 95% CI: [+6.9597E+02,+7.4300E+02]
  +6.38129521303673E+02 .*                N5_cv  + ... % SE: 1.1944E+01, PE:   1.87%, 95% CI: [+6.1424E+02,+6.6202E+02]
  +6.33457273529289E+02 .*                N3_cv  + ... % SE: 1.1774E+01, PE:   1.86%, 95% CI: [+6.0991E+02,+6.5700E+02]
  +3.20753529772084E+02 .*         u_cv .* w_cv  + ... % SE: 1.3517E+01, PE:   4.21%, 95% CI: [+2.9372E+02,+3.4779E+02]
  +1.85900582217132E+02 .*             N7_cv.^2  + ... % SE: 2.9392E+01, PE:  15.81%, 95% CI: [+1.2712E+02,+2.4468E+02]
  +1.28270071851636E+02 .*                LA_cv  + ... % SE: 1.1797E+01, PE:   9.20%, 95% CI: [+1.0468E+02,+1.5186E+02]
  +1.30014290011250E+02 .*        u_cv .* N4_cv  + ... % SE: 1.3614E+01, PE:  10.47%, 95% CI: [+1.0279E+02,+1.5724E+02]
  +1.81612145037772E+02 .*             N3_cv.^2  + ... % SE: 2.9738E+01, PE:  16.37%, 95% CI: [+1.2214E+02,+2.4109E+02]
  +1.36659753584708E+02 .*        u_cv .* N6_cv  + ... % SE: 1.3587E+01, PE:   9.94%, 95% CI: [+1.0949E+02,+1.6383E+02]
  +1.16689782223692E+02 .*                RA_cv  + ... % SE: 1.1763E+01, PE:  10.08%, 95% CI: [+9.3164E+01,+1.4022E+02]
  +1.09904301372254E+02 .*        w_cv .* N6_cv  + ... % SE: 1.3582E+01, PE:  12.36%, 95% CI: [+8.2740E+01,+1.3707E+02]
  +9.90311039275785E+01 .*        w_cv .* N4_cv  + ... % SE: 1.3674E+01, PE:  13.81%, 95% CI: [+7.1683E+01,+1.2638E+02]
  -3.36872973647805E+02 .*              w_cv.^2  + ... % SE: 2.9888E+01, PE:   8.87%, 95% CI: [-3.9665E+02,-2.7710E+02]
  +1.61056748315633E+02 .*             N2_cv.^2  + ... % SE: 2.9312E+01, PE:  18.20%, 95% CI: [+1.0243E+02,+2.1968E+02]
  +1.37426509478769E+02 .*             N8_cv.^2  + ... % SE: 2.9614E+01, PE:  21.55%, 95% CI: [+7.8198E+01,+1.9666E+02]
  +8.53396868123899E+01 .*        u_cv .* N2_cv  + ... % SE: 1.3665E+01, PE:  16.01%, 95% CI: [+5.8011E+01,+1.1267E+02]
  +7.97190088384760E+01 .*        u_cv .* N8_cv  + ... % SE: 1.3615E+01, PE:  17.08%, 95% CI: [+5.2490E+01,+1.0695E+02]
  +7.42762048044546E+01 .*        v_cv .* N3_cv  + ... % SE: 1.3587E+01, PE:  18.29%, 95% CI: [+4.7103E+01,+1.0145E+02]
  +7.41925623172130E+01 .*        u_cv .* RA_cv  + ... % SE: 1.3591E+01, PE:  18.32%, 95% CI: [+4.7011E+01,+1.0137E+02]
  +7.08401196924528E+01 .*        u_cv .* LA_cv  + ... % SE: 1.3567E+01, PE:  19.15%, 95% CI: [+4.3705E+01,+9.7975E+01]
  +1.28844962611167E+02 .*             N4_cv.^2  + ... % SE: 2.9328E+01, PE:  22.76%, 95% CI: [+7.0190E+01,+1.8750E+02]
  +1.27428538849978E+02 .*             N6_cv.^2 ;      % SE: 2.9488E+01, PE:  23.14%, 95% CI: [+6.8454E+01,+1.8640E+02]

% Fdrag Model
RSQ(8)=99.157; PSE(8)=74527; PRESS(8)=3.5844E+07; NTERMS(8)=17;
Fdrag = ...
  +2.90344542425375E+03 .*                 w_cv  + ... % SE: 1.0805E+01, PE:   0.37%, 95% CI: [+2.8818E+03,+2.9251E+03]
  +3.95717656261927E+02 .*                    1  + ... % SE: 1.4390E+01, PE:   3.64%, 95% CI: [+3.6694E+02,+4.2450E+02]
  -9.08385137944767E+02 .*         u_cv .* w_cv  + ... % SE: 1.2478E+01, PE:   1.37%, 95% CI: [-9.3334E+02,-8.8343E+02]
  +3.23638285024368E+02 .*        w_cv .* N2_cv  + ... % SE: 1.2524E+01, PE:   3.87%, 95% CI: [+2.9859E+02,+3.4869E+02]
  +3.38220493802001E+02 .*        w_cv .* N6_cv  + ... % SE: 1.2532E+01, PE:   3.71%, 95% CI: [+3.1316E+02,+3.6329E+02]
  +3.32653873337336E+02 .*        w_cv .* N8_cv  + ... % SE: 1.2546E+01, PE:   3.77%, 95% CI: [+3.0756E+02,+3.5775E+02]
  +3.41275199876057E+02 .*        w_cv .* N4_cv  + ... % SE: 1.2594E+01, PE:   3.69%, 95% CI: [+3.1609E+02,+3.6646E+02]
  +2.45537653150739E+02 .*        w_cv .* N1_cv  + ... % SE: 1.2500E+01, PE:   5.09%, 95% CI: [+2.2054E+02,+2.7054E+02]
  +2.32097929415364E+02 .*        w_cv .* N7_cv  + ... % SE: 1.2512E+01, PE:   5.39%, 95% CI: [+2.0707E+02,+2.5712E+02]
  +3.57669883990841E+02 .*              w_cv.^2  + ... % SE: 1.9856E+01, PE:   5.55%, 95% CI: [+3.1796E+02,+3.9738E+02]
  +2.17578210913018E+02 .*        w_cv .* N5_cv  + ... % SE: 1.2634E+01, PE:   5.81%, 95% CI: [+1.9231E+02,+2.4285E+02]
  +2.15788163203217E+02 .*        w_cv .* N3_cv  + ... % SE: 1.2475E+01, PE:   5.78%, 95% CI: [+1.9084E+02,+2.4074E+02]
  +1.38986796659425E+02 .*                 u_cv  + ... % SE: 1.0841E+01, PE:   7.80%, 95% CI: [+1.1730E+02,+1.6067E+02]
  +9.48813058323239E+01 .*        v_cv .* N3_cv  + ... % SE: 1.2555E+01, PE:  13.23%, 95% CI: [+6.9771E+01,+1.1999E+02]
  +6.94490899746495E+01 .*                N2_cv  + ... % SE: 1.0925E+01, PE:  15.73%, 95% CI: [+4.7600E+01,+9.1299E+01]
  +7.26901938577641E+01 .*             N8_cv.^3  + ... % SE: 1.1833E+01, PE:  16.28%, 95% CI: [+4.9024E+01,+9.6356E+01]
  +5.91813856250063E+01 .*             N4_cv.^3 ;      % SE: 1.1890E+01, PE:  20.09%, 95% CI: [+3.5402E+01,+8.2961E+01]

% Modeling Metrics
Metrics.RSQ=RSQ;
Metrics.PSE=PSE;
Metrics.PRESS=PRESS;
Metrics.NTERMS=NTERMS;

C = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];

return