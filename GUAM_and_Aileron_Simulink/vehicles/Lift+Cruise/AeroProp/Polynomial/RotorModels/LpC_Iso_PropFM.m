function [Txb_k,Tyb_k,Tzb_k,Qxb_k,Qyb_k,Qzb_k,Tpf,Qpf,Tph,Qph,CTf,CQf]=LpC_Iso_PropFM(J,Mtip,n_revps,i_deg,Prop_flow_dir_deg,rho,current_cg_location,motor_loc,Model)
% LpC_Iso_PropFM - calculate propeller forces and moments for L+C
%
% DESCRIPTION: 
%   This function calculates the forces and moments generated by the
%   propellers using the isolated propeller aerodynamics models.
%   Calculations are made using the local propeller flow conditions.
% 
% INPUTS:
%   J - 9x1 vector containing propeller total advance ratio
%   Mtip - 9x1 vector containing propeller tip Mach number
%   n_revps - 9x1 vector containing propeller speed [rev/s]
%   i_deg - 9x1 vector containing propeller incidence angle [deg]
%   Prop_flow_dir_deg - 9x1 vector containing local velocity projection
%       angle on the propeller disk plane [deg]
%   Rho - air density [slug/ft^3]
%   current_cg_location - 3x1 vectors containing the aircraft cg location
%       [ft]
%   motor_loc - 8x3 vector containing the current propeller center
%       locations [ft]
%   Model - model parameters
%
% OUTPUTS:
%   Txb_k - x body-axis component of propeller thrust for each propeller
%           [lbf]
%   Tyb_k - y body-axis component of propeller thrust for each propeller
%           [lbf]
%   Tzb_k - z body-axis component of propeller thrust for each propeller
%           [lbf]
%   Qxb_k - x body-axis component of propeller torque for each propeller
%           [ft-lbf]
%   Qyb_k - y body-axis component of propeller torque for each propeller
%           [ft-lbf]
%   Qzb_k - z body-axis component of propeller torque for each propeller
%           [ft-lbf]
%   Tpf - propeller thrust components in the local propeller flow frame
%         [lbf]
%   Qpf - propeller torque components in the local propeller flow frame
%         [ft-lbf]
%   Tpw - propeller thrust components in the wing frame [lbf]
%   Qpw - propeller torque components in the wing frame [ft-lbf]
%   CTf - propeller thrust coefficients
%   CQf - propeller torque coefficients
%
% CALLS:
%   PropAero_Interp
%
% WRITTEN BY:
%   Benjamin M. Simmons
%   NASA Langley Research Center
%   Email: benjamin.m.simmons@nasa.gov
%
% HISTORY:
%   December 12, 2019 - created and debugged LA8_PropFM.m, BMS
%   April 14, 2020 - updated for LA-8 simulation release v0.1, BMS
%   February 23, 2021 - Updated "LA8_PropFM.m" script for L+C, BMS
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

np=9;%number of propellers

% initialize propeller axis forces and moments
Txb_k=zeros(1,np);
Tyb_k=zeros(1,np);
Tzb_k=zeros(1,np);
Qxb_k=zeros(1,np);
Qyb_k=zeros(1,np);
Qzb_k=zeros(1,np);

% initialize propeller thrust and torque coefficients in the local propeller flow axes
CTf=zeros(3,np);
CQf=zeros(3,np);

% initialize propeller thrust and torque in the local propeller flow axes
Tpf=zeros(3,np);
Qpf=zeros(3,np);

% initialize propeller thrust and torque in the hub frame
Tph=zeros(3,np);
Qph=zeros(3,np);

% loop through each individual propeller
for pk=1:np
    C=zeros(1,6);% pre-allocation needed for Simulink implementation

    % calculate normal and edgewise advance ratio
    Jx=J(pk)*cosd(i_deg(pk));
    Jz=J(pk)*sind(i_deg(pk));
    
    if n_revps(pk)>0.01
    % calculate force and moment coefficients 
        if pk<=8 && pk>=1
            % check the sign of Jz, if it is negative take the absolute
            % value, and multiply the non-axial forces and moments by -1
            % later
            if Jz<0
                Jz_sign=-1;
                Jz=abs(Jz);
            else
                Jz_sign=+1;
            end
            
            % check the factor limits on Jx and Jz, if they are exceeded
            % hold the boundary value
            Jx=check_fac_limits(Jx, -0.2, 0.2, 1e12);
            Jz=check_fac_limits(Jz, 0, 1.9, 1e12);
            
            % isolated rotor model
            [C]=LpC_CCW_RotorModel_J040914(Jx,Jz,Mtip(pk));
            % C = [C_Tx,C_Ty,C_Tz,C_Qx,C_Qy,C_Qz]
            
            % change the signs of C_Ty, C_Tz, C_Qy, and C_Qz if Jz is
            % negative
            C(:,[2,3,5,6])=C(:,[2,3,5,6])*Jz_sign;

            % Since the isolated rotor model was developed for a CCW rotor, the
            % signs for CTy, CQx, and CQz need to be reversed for the CW
            % rotating propeller
            % NOTE in Model.prop_spin: (CW=+1, CCW=-1)
            if Model.prop_spin(pk)==1
                C(:,[2,4,6])=-1*C(:,[2,4,6]);
            end

        elseif pk==9
            
            % check the factor limits on Jx, if they are exceeded
            % hold a boundary value
            Jx=check_fac_limits(Jx, -2.25, 2.25, 1e12);
            
            % isolated pusher prop model
            [CT,CQ]=LpC_PusherProp_Model(Jx);
            C=[CT,0,0,CQ,0,0];
        else
            error('incorrect propeller index')
        end

        % extract propeller thrust and torque coefficients
        C_Tx=C(1);
        C_Ty=C(2);
        C_Tz=C(3);
        C_Qx=C(4);
        C_Qy=C(5);
        C_Qz=C(6);

    else
        % forces and moments are zero if there is no rotor speed
        C_Tx=0;
        C_Ty=0;
        C_Tz=0;
        C_Qx=0;
        C_Qy=0;
        C_Qz=0;
    end
        
    CTf(:,pk)=[C_Tx;C_Ty;C_Tz];
    CQf(:,pk)=[C_Qx;C_Qy;C_Qz];
    
    % Propeller Forces in the local flow relative frame [lbf]
    Tpf(1,pk)=C_Tx*(rho.*n_revps(pk).^2.*Model.Prop_D(pk)^4);
    Tpf(2,pk)=C_Ty*(rho.*n_revps(pk).^2.*Model.Prop_D(pk)^4);
    Tpf(3,pk)=C_Tz*(rho.*n_revps(pk).^2.*Model.Prop_D(pk)^4);
    
    % Propeller Moments in the local flow relative frame [ft-lbf]
    Qpf(1,pk)=C_Qx*(rho.*n_revps(pk).^2.*Model.Prop_D(pk)^5);
    Qpf(2,pk)=C_Qy*(rho.*n_revps(pk).^2.*Model.Prop_D(pk)^5);
    Qpf(3,pk)=C_Qz*(rho.*n_revps(pk).^2.*Model.Prop_D(pk)^5);
    
    % rotation matrix from the direction of relative flow at the propeller 
    % disk to the hub frame
    R_HF=[1, 0,                      0
          0, cosd(Prop_flow_dir_deg(pk)),sind(Prop_flow_dir_deg(pk))
          0, -sind(Prop_flow_dir_deg(pk)),cosd(Prop_flow_dir_deg(pk))];
      
    % transform the forces and moments into the wing frame
    Tph(:,pk)= R_HF*Tpf(:,pk);
    Qph(:,pk)= R_HF*Qpf(:,pk);
    
    % rotation matrix from the hub frame to the fuselage body frame
    R_bh=Model.Prop_R_BH(1:3,1:3,pk);
    
    % transform the propeller forces and moments into the body frame
    Tpb=R_bh*Tph(:,pk);
    Txpb=Tpb(1);% x body-axis force [lbf]
    Typb=Tpb(2);% y body-axis force [lbf]
    Tzpb=Tpb(3);% z body-axis force [lbf]
    Qpb=R_bh*Qph(:,pk);
    Qxpb=Qpb(1);% x body-axis force [ft-lbf]
    Qypb=Qpb(2);% y body-axis force [ft-lbf]
    Qzpb=Qpb(3);% z body-axis force [ft-lbf]
    
    % distance from the current CG location to the motor locations
    % this is used to transfer the propeller moments to the CG location
    dxyz=current_cg_location-motor_loc(:,pk)';
    dx_m=dxyz(1);
    dy_m=dxyz(2);
    dz_m=dxyz(3);
    
    % propeller moments at the CG in the aircraft body frame [ft-lbf]
    Qxb_k(pk)=Qxpb+Typb*dz_m-Tzpb*dy_m;
    Qyb_k(pk)=Qypb+Tzpb*dx_m-Txpb*dz_m;
    Qzb_k(pk)=Qzpb+Txpb*dy_m-Typb*dx_m;
    
    % propeller forces in the body frame [lbf]
    Txb_k(pk)=Txpb;
    Tyb_k(pk)=Typb;
    Tzb_k(pk)=Tzpb;

end
