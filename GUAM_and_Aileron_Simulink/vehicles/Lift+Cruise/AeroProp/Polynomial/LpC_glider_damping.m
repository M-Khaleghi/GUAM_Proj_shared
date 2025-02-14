function [Xd,Yd,Zd,Ld,Md,Nd] = LpC_glider_damping(V_fps,rho,p_rps,q_rps,r_rps,Model)
% LpC_glider_dyn_derivatives - dynamic derivatives for the L+C simulation
%
% DESCRIPTION: 
%   This code computes dynamic derivatives used for the Lift+Cruise simulation.
%   Currently dynamic derivatives are values are estimated from the
%   semi-empirical strip theory model (s-function model) using only the
%   bare-airframe without propellers. Propeller damping are captured using
%   isolate rotor models.
% 
% INPUTS:
%   V_fps - true airspeed [ft/s]
%   rho - air density [slug/ft^3]
%   p_rps - roll rate [rad/s]
%   q_rps - pitch rate [rad/s]
%   r_rps - yaw rate [rad/s]
%   Model - model parameters
%
% OUTPUTS:
%   [Xd,Yd,Zd,Ld,Md,Nd] - Forces and moments attributed to the dynamic
%                         derivatives [lbf] or [ft-lbf]
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
%   February 09, 2020 - created and debugged, BMS
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
% ***FOR INTERNAL NASA USE ONLY***
%

% compute the dynamic pressure [psf]
qbar=1/2.*rho.*V_fps.^2;

% Lift+Cruise geometry used for normalization
% S=231.49; b=47.72; cbar=3.94;
S=Model.S;% wing area [ft^2]
b=Model.b;% wing span [ft]
cbar=Model.cbar;% mean aerodynamic chord [ft]

% Lift+Cruise airframe-only damping derivatives.
%
% Nominal flight conditions:
%  - freestream velocity = 110 kts
%  - angle of attack = 0 deg
%  - angle of sideslip = 0 deg
%  - altitude = 6000 ft (rho = 0.00198698 slug/ft^3)
%  - control surfaces deflections = 0 deg
%  - rotor/propeller speed = 0 rpm
%
% Assume airframe damping effects can be superimposed with propeller-only
% damping. Interactional damping is assumed to be negligible.
%
% Future, more rigorous damping studies for the Lift+Cruise are desired.
%

% damping estimates using a cambered airfoil similar to the Lift+Cruise airfoil
Cyp = -0.1042;
Clp = -0.678;
Cnp = -0.000311;

Cxq = -0.215;
Czq = -0.787;
Cmq = -25.52;

Cyr = +0.378;
Clr = +0.1600;
Cnr = -0.1114;


% estimates using the generic NACA 0015 S-function airfoil
% Cyp = -0.0917;
% Clp = -0.8200;
% Cnp = +0.0531;
% 
% Cxq = +0.0529;
% Czq = -1.777;
% Cmq = -26.056;
% 
% Cyr = +0.3984;
% Clr = +0.0531;
% Cnr = -0.1141;


if qbar>0.01

    % dimensionless angular rates (using aircraft parameters)
    phat=p_rps*b/(2*V_fps);
    qhat=q_rps*cbar/(2*V_fps);
    rhat=r_rps*b/(2*V_fps);

    % dynamic derivative contributions to the force and moment coefficients
    CX=Cxq*qhat;
    CZ=Czq*qhat;
    Cm=Cmq*qhat;
    CY=Cyp*phat+Cyr*rhat;
    Cl=Clp*phat+Clr*rhat;
    Cn=Cnp*phat+Cnr*rhat;

    % dynamic derivative contributions to the dimensional forces and moments
    Xd=qbar*S*CX;
    Yd=qbar*S*CY;
    Zd=qbar*S*CZ;
    Ld=qbar*S*b*Cl;
    Md=qbar*S*cbar*Cm;
    Nd=qbar*S*b*Cn;  

else
    % dynamic derivative contributions to the dimensional forces and moments
    Xd=0;
    Yd=0;
    Zd=0;
    Ld=0;
    Md=0;
    Nd=0;  
end

return