function NormalLoads = SteadyStateWeightTransfer( LongAcc, LatAcc, ...
    SprungMass, SprungCG, UnsprungMass, UnsprungCG, ...
    RollStiffness, RollCenter, PitchStiffness, PitchCenter, ...
    Wheelbase, TrackWidth )

%% SteadyStateWeightTransfer - Simplified Weight Transfer Evaluation
% Script evaluates steady state weight transfer accounting for pitch and
% roll center on an even racing surface. The time response discrepancies of
% body roll and jacking weight transfer are ignored.
% 
% Inputs:
%   LongAcc       - (n,1   numeric) Longitudinal Acceleration      {a_x}                 [m/s^2]
%   LatAcc        - (n,1   numeric) Lateral Acceleration           {a_y}                 [m/s^2]
%   SprungMass    - (n,1   numeric) Sprung Mass                    {m_s}                 [kg]
%   SprungCG      - (n,3   numeric) Sprung Mass CG                 {[x,y,z]_{s,cg}}      [m]
%   UnsprungMass  - (n,2   numeric) [Front, Rear] Unsprung Mass    {m_{u,(f|r)}}         [kg] 
%   UnsprungCG    - (n,2,3 numeric) [Front, Rear] Unsprung Mass CG {[x,y,z]_{u(f|r),cg}} [m]
%   RollStiffness - (n,2   numeric) [Front, Rear] Roll Stiffness   {k_{phi,(f|r)}        [N-m/rad]
%   RollCenter    - (n,2,3 numeric) [Front, Rear] Roll Center      {[x,y,z]_{i,rc}}      [m]
%   Wheelbase     - (n,1   numeric) Wheelbase                      {L}                   [m]
%   TrackWidth    - (n,2   numeric) [Front, Rear] Track Width      {t_{w,(f|r)}          [m]
%
% Outputs:
%   NormalLoad - (n,4 numeric) Normal Load {F_z} [N]
%
% Notes:
%
% Author(s): 
% Blake Christierson (bechristierson@ucdavis.edu) [Sep 2018 - Jun 2021] 
% 
% Last Updated: 22-Apr-2021

%% Test Case 
if nargin == 0
    LongAcc = -0.6 .* 9.81;
    LatAcc  =  0.7 .* 9.81;
    
    SprungMass = 
end

%% Longitudinal Weight Transfer

%% Lateral Weight Transfer
SprungRollArm = SprungCG(:,3) -  
LatLoadTrans(1) = RollStiffness(1) .* (