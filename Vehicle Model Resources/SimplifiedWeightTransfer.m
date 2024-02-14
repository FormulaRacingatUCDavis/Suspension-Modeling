function [NormalLoad] = SimplifiedWeightTransfer( LongAccTot, LatAccTot, ...
        Wheelbase, TrackWidth, Mass, CoG, Downforce, CoP, PerLLT ) 
%% SimplifiedWeightTransfer - Simplified Normal Loads
% Computes tire normal loads using simplified weight transfer and
% aerodynamic loading.
% 
% Inputs:
%   LongAccTot - (n,1 numeric) Total Longitudinal Acceleration     {a_x}   [m/s^2]
%   LatAccTot  - (n,1 numeric) Total Lateral Acceleration          {a_y}   [m/s^2]
%   Wheelbase  - (n,1 numeric) Wheelbase                           {L}     [m]
%   TrackWidth - (n,2 numeric) Track Width                         {t_w}   [m]
%   Mass       - (n,1 numeric) Total Vehicle Mass                  {m}     [kg]
%   CoG        - (n,3 numeric) Center of Gravity                   {CoG}   [m]
%   Downforce  - (n,1 numeric) Aerodynamic Downforce               {A_F_z} [N]
%   CoP        - (n,3 numeric) Center of Pressure                  {CoP}   [m]
%   PerLLT     - (n,1 numeric) Percent Front Lateral Load Transfer {%_LLT} [ ]
%
% Outputs:
%   NormalLoad - (n,4 numeric) Tire Normal Loads                   {T_F_z} [N]
%
% Notes:
%
% Author(s): 
% Blake Christierson (bechristierson@ucdavis.edu) [Sep 2018 - Jun 2021] 
% 
% Last Updated: 30-May-2021

%% Test Case
if nargin == 0
    LongAccTot = 1.3 .* 9.81;
    LatAccTot  =  1.3 .* 9.81;
    
    Wheelbase = 1.575;
    TrackWidth = [1.20, 1.20];
    
    Mass = 280;
    CoG  = [(0.5-0.5)*Wheelbase, 0, 0.25];
    
    Downforce = 0; %200;
    CoP  = [(0.4-0.5)*Wheelbase, 0, 0];
    
    PerLLT = 0.5;
    
    [NormalLoad] = SimplifiedWeightTransfer( LongAccTot, LatAccTot, ...
        Wheelbase, TrackWidth, Mass, CoG, Downforce, CoP, PerLLT ) %#ok<NOPRT>
    
    return
end

%% Computation
NormalLoad = zeros( [size( LongAccTot ), 4] );

%%% Total Weight Transfer
dFzx = Mass .* LongAccTot .* CoG(:,3) ./ Wheelbase;
dFzy = Mass .* LatAccTot  .* CoG(:,3) ./ mean(TrackWidth, 2);

%%% Static Wheel Loads
MassBal = (CoG(:,1) + Wheelbase./2) ./ Wheelbase;
AeroBal = (CoP(:,1) + Wheelbase./2) ./ Wheelbase;

NormalLoad(:,1:2) = (MassBal .* Mass .* 9.81 + AeroBal .* Downforce) / 2;
NormalLoad(:,3:4) = ((1-MassBal) .* Mass .* 9.81 + (1-AeroBal) .* Downforce) / 2;

%%% Dynamic Wheel Loads
NormalLoad(:,1) = NormalLoad(:,1) + (-0.5.*dFzx - PerLLT    .*dFzy)/2;
NormalLoad(:,2) = NormalLoad(:,2) + (-0.5.*dFzx + PerLLT    .*dFzy)/2;
NormalLoad(:,3) = NormalLoad(:,3) + ( 0.5.*dFzx - (1-PerLLT).*dFzy)/2;
NormalLoad(:,4) = NormalLoad(:,4) + ( 0.5.*dFzx + (1-PerLLT).*dFzy)/2;

end

