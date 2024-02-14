function [] = RollSteer( ) 
%% Non-Rigid Lateral Load Weight Transfer - 
% Computes tire normal loads using Non-rigid chassis model by Sampo
% 
% Inputs:
%   LatAcc     - (n,1 numeric) Total Lateral Acceleration         {a_x}        [m/s^2]
%
% Outputs:
%   delta_Fz_F - (n,1 numeric) Front Lateral Load Weight Transfer {DeltaF_zF}} [N]
%   delta_Fz_R - (n,1 numeric) Rear Lateral Load Weight Transfer  {DeltaF_zR}} [N]
%
% Notes:
%
% Author(s): 
% Tristan Pham (atlpham@ucdavis.edu) [Sep 2020 - Present] 
% 
% Last Updated: 31-August-2023

%% Test Case
if nargin == 0
    m = 290;
    m_s = 225;
    m_uF = 28;
    
   

%% Computation
m_sF = b_s./l;


end