function [C,Metrics]=LCGabCFMs0p2__Cp3FI_MOF_cv(u,Beta,Alpha,LA,RA,LE,RE,RUD)
% LCGabCFMs0p2__Cp3FI_MOF_cv - Aerodynamic model for the LpC Glider
%
% DESCRIPTION: 
%   This script contains the aerodynamic model for LpC Glider. The script was
%   automatically generated using "GenModelCV.m" on 14-Jan-2021 17:08:43
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

%   Modeling Algorithm: MOF, Model Complexity: Cubic+3FI 
%   Model SF: 0.2, SWR Alpha: 1 
%   Final Fval: [  0  0  0  0  0  0  0  0  ] 
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
RSQ(1)=99.609; PSE(1)=4.2193E-05; PRESS(1)=0.017027; NTERMS(1)=12;
CFx = ...
  -9.65243531820972E-02 .*             Alpha_cv  + ... % SE: 2.4158E-04, PE:   0.25%, 95% CI: [-9.7008E-02,-9.6041E-02]
  -1.59861230214132E-02 .*    Alpha_cv .* RA_cv  + ... % SE: 2.9203E-04, PE:   1.83%, 95% CI: [-1.6570E-02,-1.5402E-02]
  -1.59757773535520E-02 .*    Alpha_cv .* LA_cv  + ... % SE: 2.9200E-04, PE:   1.83%, 95% CI: [-1.6560E-02,-1.5392E-02]
  -1.18335375258893E-02 .*                RA_cv  + ... % SE: 2.3837E-04, PE:   2.01%, 95% CI: [-1.2310E-02,-1.1357E-02]
  -1.18038960509986E-02 .*                LA_cv  + ... % SE: 2.4048E-04, PE:   2.04%, 95% CI: [-1.2285E-02,-1.1323E-02]
  +1.85621462568497E-02 .*             RA_cv.^2  + ... % SE: 4.7591E-04, PE:   2.56%, 95% CI: [+1.7610E-02,+1.9514E-02]
  -3.30659488793303E-02 .*          Alpha_cv.^2  + ... % SE: 4.6835E-04, PE:   1.42%, 95% CI: [-3.4003E-02,-3.2129E-02]
  +1.88132496785347E-02 .*             LA_cv.^2  + ... % SE: 4.8013E-04, PE:   2.55%, 95% CI: [+1.7853E-02,+1.9774E-02]
  -1.78511872187146E-02 .*                    1  + ... % SE: 3.8013E-04, PE:   2.13%, 95% CI: [-1.8611E-02,-1.7091E-02]
  +1.09692157139401E-02 .*            RUD_cv.^2  + ... % SE: 4.6953E-04, PE:   4.28%, 95% CI: [+1.0030E-02,+1.1908E-02]
  +7.50246667262807E-03 .*             LE_cv.^2  + ... % SE: 4.8051E-04, PE:   6.40%, 95% CI: [+6.5414E-03,+8.4635E-03]
  +7.09284889706597E-03 .*             RE_cv.^2 ;      % SE: 4.7265E-04, PE:   6.66%, 95% CI: [+6.1475E-03,+8.0382E-03]

% CFy Model
RSQ(2)=99.257; PSE(2)=3.7891E-05; PRESS(2)=0.016718; NTERMS(2)=20;
CFy = ...
  -6.10394115924635E-02 .*               RUD_cv  + ... % SE: 9.7723E-04, PE:   1.60%, 95% CI: [-6.2994E-02,-5.9085E-02]
  -3.69865673503009E-02 .*              Beta_cv  + ... % SE: 2.3515E-04, PE:   0.64%, 95% CI: [-3.7457E-02,-3.6516E-02]
  +1.11555449359847E-02 .*                RA_cv  + ... % SE: 2.3436E-04, PE:   2.10%, 95% CI: [+1.0687E-02,+1.1624E-02]
  -1.09612433783234E-02 .*                LA_cv  + ... % SE: 2.3648E-04, PE:   2.16%, 95% CI: [-1.1434E-02,-1.0488E-02]
  +6.85440326977356E-03 .*   Alpha_cv .* RUD_cv  + ... % SE: 2.8931E-04, PE:   4.22%, 95% CI: [+6.2758E-03,+7.4330E-03]
  +4.07018170646957E-03 .*       u_cv .* RUD_cv  + ... % SE: 3.6422E-04, PE:   8.95%, 95% CI: [+3.3417E-03,+4.7986E-03]
  +1.87532408036567E-03 .*    LA_cv.^2 .* RE_cv  + ... % SE: 5.7691E-04, PE:  30.76%, 95% CI: [+7.2149E-04,+3.0292E-03]
  -7.35665576495683E-03 .*                LE_cv  + ... % SE: 4.5338E-04, PE:   6.16%, 95% CI: [-8.2634E-03,-6.4499E-03]
  +7.62733975069496E-03 .*   LE_cv .* RUD_cv.^2  + ... % SE: 5.6872E-04, PE:   7.46%, 95% CI: [+6.4899E-03,+8.7648E-03]
  -5.51531454022515E-03 .* Beta_cv.^2 .* RUD_cv  + ... % SE: 5.7648E-04, PE:  10.45%, 95% CI: [-6.6683E-03,-4.3623E-03]
  +3.04882296279777E-03 .* LE_cv .* RE_cv .* RUD_cv  + ... % SE: 3.3452E-04, PE:  10.97%, 95% CI: [+2.3798E-03,+3.7179E-03]
  -2.75033044581526E-03 .* Beta_cv .* RE_cv .* RUD_cv  + ... % SE: 3.3486E-04, PE:  12.18%, 95% CI: [-3.4201E-03,-2.0806E-03]
  -3.27765735274265E-03 .* u_cv .* Beta_cv .* Alpha_cv  + ... % SE: 4.3265E-04, PE:  13.20%, 95% CI: [-4.1430E-03,-2.4124E-03]
  -6.25163218510429E-03 .*   RE_cv .* RUD_cv.^2  + ... % SE: 5.6377E-04, PE:   9.02%, 95% CI: [-7.3792E-03,-5.1241E-03]
  +5.50605204219734E-03 .*                RE_cv  + ... % SE: 5.6249E-04, PE:  10.22%, 95% CI: [+4.3811E-03,+6.6310E-03]
  -1.92649628765694E-03 .*  Beta_cv .* Alpha_cv  + ... % SE: 2.8462E-04, PE:  14.77%, 95% CI: [-2.4957E-03,-1.3573E-03]
  +6.13418732014842E-03 .*            RUD_cv.^3  + ... % SE: 1.0323E-03, PE:  16.83%, 95% CI: [+4.0697E-03,+8.1987E-03]
  +2.47692526894159E-03 .* u_cv .* RE_cv .* RUD_cv  + ... % SE: 4.3580E-04, PE:  17.59%, 95% CI: [+1.6053E-03,+3.3485E-03]
  +1.80674498649723E-03 .* Beta_cv .* LE_cv .* RUD_cv  + ... % SE: 3.3537E-04, PE:  18.56%, 95% CI: [+1.1360E-03,+2.4775E-03]
  +1.50708309215948E-03 .*      LA_cv .* RUD_cv ;      % SE: 2.8800E-04, PE:  19.11%, 95% CI: [+9.3108E-04,+2.0831E-03]

% CFz Model
RSQ(3)=99.892; PSE(3)=0.0011079; PRESS(3)=0.18004; NTERMS(3)=14;
CFz = ...
  +7.54887929911287E-01 .*                    1  + ... % SE: 1.1834E-03, PE:   0.16%, 95% CI: [+7.5252E-01,+7.5725E-01]
  +5.01553903130244E-01 .*             Alpha_cv  + ... % SE: 7.8539E-04, PE:   0.16%, 95% CI: [+4.9998E-01,+5.0312E-01]
  +3.37435335454543E-01 .*                LA_cv  + ... % SE: 3.0455E-03, PE:   0.90%, 95% CI: [+3.3134E-01,+3.4353E-01]
  +3.32940562243718E-01 .*                RA_cv  + ... % SE: 3.1554E-03, PE:   0.95%, 95% CI: [+3.2663E-01,+3.3925E-01]
  +5.54476797776374E-02 .*                LE_cv  + ... % SE: 7.7742E-04, PE:   1.40%, 95% CI: [+5.3893E-02,+5.7003E-02]
  +5.39059739888616E-02 .*                RE_cv  + ... % SE: 7.7702E-04, PE:   1.44%, 95% CI: [+5.2352E-02,+5.5460E-02]
  -3.36125920368057E-02 .*             RA_cv.^2  + ... % SE: 1.4931E-03, PE:   4.44%, 95% CI: [-3.6599E-02,-3.0626E-02]
  -3.30970430278900E-02 .*             LA_cv.^2  + ... % SE: 1.5302E-03, PE:   4.62%, 95% CI: [-3.6157E-02,-3.0037E-02]
  -7.80422987986182E-02 .*             LA_cv.^3  + ... % SE: 3.2655E-03, PE:   4.18%, 95% CI: [-8.4573E-02,-7.1511E-02]
  -2.52018946100476E-02 .*           Beta_cv.^2  + ... % SE: 1.4973E-03, PE:   5.94%, 95% CI: [-2.8196E-02,-2.2207E-02]
  -7.17366001513492E-02 .*             RA_cv.^3  + ... % SE: 3.3484E-03, PE:   4.67%, 95% CI: [-7.8433E-02,-6.5040E-02]
  -2.71386504007096E-02 .*          Alpha_cv.^2  + ... % SE: 1.4998E-03, PE:   5.53%, 95% CI: [-3.0138E-02,-2.4139E-02]
  -1.57923464165089E-02 .*    Alpha_cv .* LA_cv  + ... % SE: 9.5148E-04, PE:   6.02%, 95% CI: [-1.7695E-02,-1.3889E-02]
  -1.48953031819480E-02 .*    Alpha_cv .* RA_cv ;      % SE: 9.4888E-04, PE:   6.37%, 95% CI: [-1.6793E-02,-1.2998E-02]

% CMx Model
RSQ(4)=99.913; PSE(4)=0.00010102; PRESS(4)=0.015616; NTERMS(4)=12;
CMx = ...
  -1.84546740828982E-01 .*                RA_cv  + ... % SE: 9.2934E-04, PE:   0.50%, 95% CI: [-1.8641E-01,-1.8269E-01]
  +1.85090223930524E-01 .*                LA_cv  + ... % SE: 8.9509E-04, PE:   0.48%, 95% CI: [+1.8330E-01,+1.8688E-01]
  -1.25656669268027E-02 .*               RUD_cv  + ... % SE: 2.3051E-04, PE:   1.83%, 95% CI: [-1.3027E-02,-1.2105E-02]
  -4.35373624908096E-02 .*             LA_cv.^3  + ... % SE: 9.6012E-04, PE:   2.21%, 95% CI: [-4.5458E-02,-4.1617E-02]
  +4.31875913778542E-02 .*             RA_cv.^3  + ... % SE: 9.8635E-04, PE:   2.28%, 95% CI: [+4.1215E-02,+4.5160E-02]
  -7.83942171496389E-03 .*              Beta_cv  + ... % SE: 2.2948E-04, PE:   2.93%, 95% CI: [-8.2984E-03,-7.3805E-03]
  -8.02528841734474E-03 .*    Alpha_cv .* LA_cv  + ... % SE: 2.7985E-04, PE:   3.49%, 95% CI: [-8.5850E-03,-7.4656E-03]
  +7.88384346882058E-03 .*    Alpha_cv .* RA_cv  + ... % SE: 2.7918E-04, PE:   3.54%, 95% CI: [+7.3255E-03,+8.4422E-03]
  -4.05941181846646E-03 .*                RE_cv  + ... % SE: 2.2881E-04, PE:   5.64%, 95% CI: [-4.5170E-03,-3.6018E-03]
  -1.74000229990479E-02 .*             LA_cv.^2  + ... % SE: 3.7857E-04, PE:   2.18%, 95% CI: [-1.8157E-02,-1.6643E-02]
  +1.70660543883971E-02 .*             RA_cv.^2  + ... % SE: 3.7311E-04, PE:   2.19%, 95% CI: [+1.6320E-02,+1.7812E-02]
  +4.12912934616792E-03 .*                LE_cv ;      % SE: 2.2826E-04, PE:   5.53%, 95% CI: [+3.6726E-03,+4.5856E-03]

% CMy Model
RSQ(5)=99.289; PSE(5)=0.00091352; PRESS(5)=0.34538; NTERMS(5)=28;
CMy = ...
  -2.50526964421504E-01 .*                LE_cv  + ... % SE: 4.2273E-03, PE:   1.69%, 95% CI: [-2.5898E-01,-2.4207E-01]
  -2.63964893499119E-01 .*                RE_cv  + ... % SE: 4.3991E-03, PE:   1.67%, 95% CI: [-2.7276E-01,-2.5517E-01]
  +5.66705689816443E-02 .*                    1  + ... % SE: 1.7055E-03, PE:   3.01%, 95% CI: [+5.3259E-02,+6.0082E-02]
  +2.40647029422051E-02 .*             Alpha_cv  + ... % SE: 2.1291E-03, PE:   8.85%, 95% CI: [+1.9806E-02,+2.8323E-02]
  +2.80968735443988E-02 .*             RE_cv.^2  + ... % SE: 2.1364E-03, PE:   7.60%, 95% CI: [+2.3824E-02,+3.2370E-02]
  +3.17120631024996E-02 .*            RUD_cv.^2  + ... % SE: 2.1020E-03, PE:   6.63%, 95% CI: [+2.7508E-02,+3.5916E-02]
  -4.72771174599906E-02 .*          Alpha_cv.^2  + ... % SE: 2.1140E-03, PE:   4.47%, 95% CI: [-5.1505E-02,-4.3049E-02]
  +3.15312680131741E-02 .*             LE_cv.^2  + ... % SE: 2.1489E-03, PE:   6.82%, 95% CI: [+2.7233E-02,+3.5829E-02]
  +1.40830741972536E-02 .*    Beta_cv .* RUD_cv  + ... % SE: 1.2930E-03, PE:   9.18%, 95% CI: [+1.1497E-02,+1.6669E-02]
  -1.47790144567024E-02 .*    Alpha_cv .* RA_cv  + ... % SE: 1.2991E-03, PE:   8.79%, 95% CI: [-1.7377E-02,-1.2181E-02]
  +1.96397784220543E-02 .*           Beta_cv.^2  + ... % SE: 2.1263E-03, PE:  10.83%, 95% CI: [+1.5387E-02,+2.3892E-02]
  -1.24836828456051E-02 .*     Beta_cv .* RE_cv  + ... % SE: 1.2806E-03, PE:  10.26%, 95% CI: [-1.5045E-02,-9.9224E-03]
  +1.30728112454185E-02 .*      LE_cv .* RUD_cv  + ... % SE: 1.2907E-03, PE:   9.87%, 95% CI: [+1.0491E-02,+1.5654E-02]
  -1.32863398269319E-02 .*    Alpha_cv .* LA_cv  + ... % SE: 1.3007E-03, PE:   9.79%, 95% CI: [-1.5888E-02,-1.0685E-02]
  +1.76558841615691E-02 .*             RA_cv.^2  + ... % SE: 2.1246E-03, PE:  12.03%, 95% CI: [+1.3407E-02,+2.1905E-02]
  +3.23536841293764E-02 .*             RE_cv.^3  + ... % SE: 4.6276E-03, PE:  14.30%, 95% CI: [+2.3098E-02,+4.1609E-02]
  +1.04359075309794E-02 .*     Beta_cv .* LE_cv  + ... % SE: 1.2772E-03, PE:  12.24%, 95% CI: [+7.8815E-03,+1.2990E-02]
  +1.39852058355435E-02 .*             LA_cv.^2  + ... % SE: 2.1460E-03, PE:  15.34%, 95% CI: [+9.6932E-03,+1.8277E-02]
  -9.76825326884439E-03 .*      RE_cv .* RUD_cv  + ... % SE: 1.2964E-03, PE:  13.27%, 95% CI: [-1.2361E-02,-7.1754E-03]
  +1.60310771915446E-02 .* Beta_cv.^2 .* Alpha_cv  + ... % SE: 2.6373E-03, PE:  16.45%, 95% CI: [+1.0756E-02,+2.1306E-02]
  +2.70898235666694E-02 .*             LE_cv.^3  + ... % SE: 4.5008E-03, PE:  16.61%, 95% CI: [+1.8088E-02,+3.6091E-02]
  +1.52752873865580E-02 .*    LE_cv.^2 .* RE_cv  + ... % SE: 2.6452E-03, PE:  17.32%, 95% CI: [+9.9850E-03,+2.0566E-02]
  +9.16817559845277E-03 .*   u_cv .* Beta_cv.^2  + ... % SE: 1.6928E-03, PE:  18.46%, 95% CI: [+5.7825E-03,+1.2554E-02]
  -9.19634413135435E-03 .*     u_cv .* Alpha_cv  + ... % SE: 1.6489E-03, PE:  17.93%, 95% CI: [-1.2494E-02,-5.8985E-03]
  +7.17573687008974E-03 .* Beta_cv .* Alpha_cv .* RUD_cv  + ... % SE: 1.5360E-03, PE:  21.40%, 95% CI: [+4.1038E-03,+1.0248E-02]
  +7.23627835435519E-03 .*       LE_cv .* RE_cv  + ... % SE: 1.2787E-03, PE:  17.67%, 95% CI: [+4.6789E-03,+9.7936E-03]
  -2.57852050704790E-02 .*   RE_cv.^2 .* RUD_cv  + ... % SE: 2.1877E-03, PE:   8.48%, 95% CI: [-3.0161E-02,-2.1410E-02]
  +2.43226276380072E-02 .*   LE_cv.^2 .* RUD_cv ;      % SE: 2.1618E-03, PE:   8.89%, 95% CI: [+1.9999E-02,+2.8646E-02]

% CMz Model
RSQ(6)=99.164; PSE(6)=1.5135E-05; PRESS(6)=0.007027; NTERMS(6)=20;
CMz = ...
  +4.28498440831928E-02 .*               RUD_cv  + ... % SE: 6.3420E-04, PE:   1.48%, 95% CI: [+4.1581E-02,+4.4118E-02]
  -7.20768480115775E-03 .*                RA_cv  + ... % SE: 1.5172E-04, PE:   2.10%, 95% CI: [-7.5111E-03,-6.9043E-03]
  +7.17610633523396E-03 .*                LA_cv  + ... % SE: 1.5320E-04, PE:   2.13%, 95% CI: [+6.8697E-03,+7.4825E-03]
  +6.46793901992802E-03 .*    Alpha_cv .* LA_cv  + ... % SE: 1.8643E-04, PE:   2.88%, 95% CI: [+6.0951E-03,+6.8408E-03]
  -6.35873012068548E-03 .*    Alpha_cv .* RA_cv  + ... % SE: 1.8594E-04, PE:   2.92%, 95% CI: [-6.7306E-03,-5.9868E-03]
  -4.61016417983929E-03 .*   Alpha_cv .* RUD_cv  + ... % SE: 1.8720E-04, PE:   4.06%, 95% CI: [-4.9846E-03,-4.2358E-03]
  +1.24069714623661E-02 .*             RA_cv.^2  + ... % SE: 2.4794E-04, PE:   2.00%, 95% CI: [+1.1911E-02,+1.2903E-02]
  -1.24490177623773E-02 .*             LA_cv.^2  + ... % SE: 2.5201E-04, PE:   2.02%, 95% CI: [-1.2953E-02,-1.1945E-02]
  -1.79022666593359E-03 .*              Beta_cv  + ... % SE: 1.5247E-04, PE:   8.52%, 95% CI: [-2.0952E-03,-1.4853E-03]
  -2.72219385286977E-03 .*       u_cv .* RUD_cv  + ... % SE: 2.3596E-04, PE:   8.67%, 95% CI: [-3.1941E-03,-2.2503E-03]
  -4.40713620604179E-03 .*                RE_cv  + ... % SE: 2.8770E-04, PE:   6.53%, 95% CI: [-4.9825E-03,-3.8317E-03]
  +4.28653555208030E-03 .*   RE_cv .* RUD_cv.^2  + ... % SE: 3.6316E-04, PE:   8.47%, 95% CI: [+3.5602E-03,+5.0129E-03]
  +3.91748615272346E-03 .* Beta_cv.^2 .* RUD_cv  + ... % SE: 3.7361E-04, PE:   9.54%, 95% CI: [+3.1703E-03,+4.6647E-03]
  -1.95311316024351E-03 .* LE_cv .* RE_cv .* RUD_cv  + ... % SE: 2.1566E-04, PE:  11.04%, 95% CI: [-2.3844E-03,-1.5218E-03]
  +4.56388620438349E-03 .*                LE_cv  + ... % SE: 2.9346E-04, PE:   6.43%, 95% CI: [+3.9770E-03,+5.1508E-03]
  -4.85230343891397E-03 .*   LE_cv .* RUD_cv.^2  + ... % SE: 3.6797E-04, PE:   7.58%, 95% CI: [-5.5882E-03,-4.1164E-03]
  +1.67439667721446E-03 .* Beta_cv .* RE_cv .* RUD_cv  + ... % SE: 2.1714E-04, PE:  12.97%, 95% CI: [+1.2401E-03,+2.1087E-03]
  -4.69290519024805E-03 .*            RUD_cv.^3  + ... % SE: 6.6894E-04, PE:  14.25%, 95% CI: [-6.0308E-03,-3.3550E-03]
  -1.59714627527136E-03 .* u_cv .* RE_cv .* RUD_cv  + ... % SE: 2.8227E-04, PE:  17.67%, 95% CI: [-2.1617E-03,-1.0326E-03]
  -1.01220103676798E-03 .*      LA_cv .* RUD_cv ;      % SE: 1.8635E-04, PE:  18.41%, 95% CI: [-1.3849E-03,-6.3951E-04]

% CL Model
RSQ(7)=99.882; PSE(7)=0.001126; PRESS(7)=0.19676; NTERMS(7)=14;
CL = ...
  +7.52141021367992E-01 .*                    1  + ... % SE: 1.2370E-03, PE:   0.16%, 95% CI: [+7.4967E-01,+7.5461E-01]
  +4.99544313314658E-01 .*             Alpha_cv  + ... % SE: 8.2096E-04, PE:   0.16%, 95% CI: [+4.9790E-01,+5.0119E-01]
  +3.37174955998871E-01 .*                LA_cv  + ... % SE: 3.1835E-03, PE:   0.94%, 95% CI: [+3.3081E-01,+3.4354E-01]
  +3.32659189421377E-01 .*                RA_cv  + ... % SE: 3.2983E-03, PE:   0.99%, 95% CI: [+3.2606E-01,+3.3926E-01]
  +5.51476249963476E-02 .*                LE_cv  + ... % SE: 8.1263E-04, PE:   1.47%, 95% CI: [+5.3522E-02,+5.6773E-02]
  +5.36366371295764E-02 .*                RE_cv  + ... % SE: 8.1222E-04, PE:   1.51%, 95% CI: [+5.2012E-02,+5.5261E-02]
  -3.57981420411090E-02 .*             RA_cv.^2  + ... % SE: 1.5607E-03, PE:   4.36%, 95% CI: [-3.8920E-02,-3.2677E-02]
  -3.51852378577273E-02 .*             LA_cv.^2  + ... % SE: 1.5995E-03, PE:   4.55%, 95% CI: [-3.8384E-02,-3.1986E-02]
  -7.77610776202383E-02 .*             LA_cv.^3  + ... % SE: 3.4134E-03, PE:   4.39%, 95% CI: [-8.4588E-02,-7.0934E-02]
  -2.56490645523889E-02 .*           Beta_cv.^2  + ... % SE: 1.5651E-03, PE:   6.10%, 95% CI: [-2.8779E-02,-2.2519E-02]
  -7.13586039308874E-02 .*             RA_cv.^3  + ... % SE: 3.5001E-03, PE:   4.90%, 95% CI: [-7.8359E-02,-6.4358E-02]
  -1.57272984594129E-02 .*    Alpha_cv .* LA_cv  + ... % SE: 9.9458E-04, PE:   6.32%, 95% CI: [-1.7716E-02,-1.3738E-02]
  -1.48254136679229E-02 .*    Alpha_cv .* RA_cv  + ... % SE: 9.9186E-04, PE:   6.69%, 95% CI: [-1.6809E-02,-1.2842E-02]
  -2.31420536880500E-02 .*          Alpha_cv.^2 ;      % SE: 1.5677E-03, PE:   6.77%, 95% CI: [-2.6277E-02,-2.0007E-02]

% CD Model
RSQ(8)=99.288; PSE(8)=1.844E-05; PRESS(8)=0.0070024; NTERMS(8)=28;
CD = ...
  +6.16671607912456E-02 .*                    1  + ... % SE: 2.5309E-04, PE:   0.41%, 95% CI: [+6.1161E-02,+6.2173E-02]
  +2.67600680572377E-02 .*             Alpha_cv  + ... % SE: 6.4121E-04, PE:   2.40%, 95% CI: [+2.5478E-02,+2.8042E-02]
  +1.77083906575537E-02 .*          Alpha_cv.^2  + ... % SE: 2.9595E-04, PE:   1.67%, 95% CI: [+1.7116E-02,+1.8300E-02]
  +2.00176719619074E-02 .*                RA_cv  + ... % SE: 6.1396E-04, PE:   3.07%, 95% CI: [+1.8790E-02,+2.1246E-02]
  +1.97606916426220E-02 .*                LA_cv  + ... % SE: 5.9575E-04, PE:   3.01%, 95% CI: [+1.8569E-02,+2.0952E-02]
  +1.48233866549031E-02 .*             RA_cv.^2  + ... % SE: 3.0016E-04, PE:   2.02%, 95% CI: [+1.4223E-02,+1.5424E-02]
  +1.53827300287274E-02 .*             LA_cv.^2  + ... % SE: 3.0267E-04, PE:   1.97%, 95% CI: [+1.4777E-02,+1.5988E-02]
  +1.02051088496587E-02 .*    Alpha_cv .* RA_cv  + ... % SE: 1.8410E-04, PE:   1.80%, 95% CI: [+9.8369E-03,+1.0573E-02]
  +1.01265162099903E-02 .*    Alpha_cv .* LA_cv  + ... % SE: 1.8446E-04, PE:   1.82%, 95% CI: [+9.7576E-03,+1.0495E-02]
  +9.86122358546247E-03 .*            RUD_cv.^2  + ... % SE: 2.9621E-04, PE:   3.00%, 95% CI: [+9.2688E-03,+1.0454E-02]
  +5.58444286233003E-03 .*    Beta_cv .* RUD_cv  + ... % SE: 1.8441E-04, PE:   3.30%, 95% CI: [+5.2156E-03,+5.9533E-03]
  +4.50512425127368E-03 .*                LE_cv  + ... % SE: 1.5039E-04, PE:   3.34%, 95% CI: [+4.2043E-03,+4.8059E-03]
  +4.29656110563053E-03 .*                RE_cv  + ... % SE: 1.5079E-04, PE:   3.51%, 95% CI: [+3.9950E-03,+4.5981E-03]
  +6.48913808492448E-03 .*             RE_cv.^2  + ... % SE: 2.9909E-04, PE:   4.61%, 95% CI: [+5.8910E-03,+7.0873E-03]
  +4.56214451436639E-03 .*    Alpha_cv .* LE_cv  + ... % SE: 1.8260E-04, PE:   4.00%, 95% CI: [+4.1969E-03,+4.9274E-03]
  +4.14774629630310E-03 .*    Alpha_cv .* RE_cv  + ... % SE: 1.8324E-04, PE:   4.42%, 95% CI: [+3.7813E-03,+4.5142E-03]
  +6.81867087414597E-03 .*             LE_cv.^2  + ... % SE: 3.0374E-04, PE:   4.45%, 95% CI: [+6.2112E-03,+7.4261E-03]
  -4.22924682444087E-03 .* Alpha_cv .* RA_cv.^2  + ... % SE: 3.8452E-04, PE:   9.09%, 95% CI: [-4.9983E-03,-3.4602E-03]
  +2.50948739856321E-03 .*     u_cv .* Alpha_cv  + ... % SE: 2.3366E-04, PE:   9.31%, 95% CI: [+2.0422E-03,+2.9768E-03]
  -3.67891973195665E-03 .* Alpha_cv .* LA_cv.^2  + ... % SE: 3.8437E-04, PE:  10.45%, 95% CI: [-4.4477E-03,-2.9102E-03]
  -5.33457443295630E-03 .*             LA_cv.^3  + ... % SE: 6.3867E-04, PE:  11.97%, 95% CI: [-6.6119E-03,-4.0572E-03]
  -5.48741559893894E-03 .*             RA_cv.^3  + ... % SE: 6.5208E-04, PE:  11.88%, 95% CI: [-6.7916E-03,-4.1833E-03]
  +2.10587696180373E-03 .*              u_cv.^2  + ... % SE: 2.9095E-04, PE:  13.82%, 95% CI: [+1.5240E-03,+2.6878E-03]
  -1.19884695864255E-03 .*       LA_cv .* RA_cv  + ... % SE: 1.8134E-04, PE:  15.13%, 95% CI: [-1.5615E-03,-8.3616E-04]
  +2.11687254516245E-03 .*  u_cv.^2 .* Alpha_cv  + ... % SE: 3.5419E-04, PE:  16.73%, 95% CI: [+1.4085E-03,+2.8252E-03]
  +4.23576631033488E-03 .*          Alpha_cv.^3  + ... % SE: 6.6804E-04, PE:  15.77%, 95% CI: [+2.8997E-03,+5.5718E-03]
  -1.05011018244091E-03 .*       LE_cv .* RE_cv  + ... % SE: 1.8216E-04, PE:  17.35%, 95% CI: [-1.4144E-03,-6.8579E-04]
  +1.49686736373317E-03 .* u_cv .* Alpha_cv .* RA_cv ;      % SE: 2.8065E-04, PE:  18.75%, 95% CI: [+9.3556E-04,+2.0582E-03]

% Modeling Metrics
Metrics.RSQ=RSQ;
Metrics.PSE=PSE;
Metrics.PRESS=PRESS;
Metrics.NTERMS=NTERMS;

C = [CFx,CFy,CFz,CMx,CMy,CMz,CL,CD];

return