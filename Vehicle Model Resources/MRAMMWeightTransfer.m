function [deltaFz_Lateral_Front,deltaFz_Lateral_Rear] = MRAMMWeightTransfer( tw,...
    unsprung_CG_height, latAcc, unsprung_mass, Pf, roll_Center_Height, suspended_mass,...
    roll_SDF,height_CG_Suspended) 
%% Initialize Parameters
if nargin == 0
    tw = 1.220;
    unsprung_CG_height = 0.220;
    latAcc = 3.0; %m/s^2 
    unsprung_mass = 77; 
    Pf = 0.50;
    roll_Center_Height = 0.1;
    suspended_mass = 191; 
    roll_SDF = .60;
    height_CG_Suspended = 0.230;
    
    [deltaFz_Lateral_Front,deltaFz_Lateral_Rear] = MRAMMWeightTransfer( tw,...
    unsprung_CG_height, latAcc, unsprung_mass, Pf, roll_Center_Height, suspended_mass,...
    roll_SDF,height_CG_Suspended) 
    return
end
%% Unsprung Weight Transfer
deltaFz_LFU = (2 .* unsprung_mass .* latAcc .* unsprung_CG_height) ./ tw; 
% Lateral Front Unsprung Weight Transfer

deltaFz_LRU = (2 .* unsprung_mass .* latAcc .* unsprung_CG_height) ./ tw; 
% Lateral Rear Unsprung Weight Transfer

%% Suspended Weight Transfer Geometric
deltaFz_Lateral_Front_Suspended_Geometric = (suspended_mass .*...
    (1-Pf) .* latAcc .* roll_Center_Height)./tw;
% Lateral Front Geometric Weight Transfer (Suspended)

deltaFz_Lateral_Rear_Suspended_Geometric = (suspended_mass .*...
    (Pf) .* latAcc .* roll_Center_Height)./tw;
% Lateral Rear Geometric Weight Transfer (Suspended)

%% Suspended Weight Transfer Elastic
deltaFz_Lateral_Front_Suspended_Elastic = ((suspended_mass .* latAcc .* (height_CG_Suspended)...
    - roll_Center_Height) ./ tw ) .* roll_SDF;
% Lateral Front Elastic Weight Transfer (Suspended)

deltaFz_Lateral_Rear_Suspended_Elastic = ((suspended_mass .* latAcc .* (height_CG_Suspended)...
    - roll_Center_Height) ./ tw ) .* (1-roll_SDF);
% Lateral Rear Elastic Transfer (Suspended)

%% Normal Force Calculation
deltaFz_Lateral_Front = deltaFz_LFU + ...
    deltaFz_Lateral_Front_Suspended_Geometric + ...
    deltaFz_Lateral_Front_Suspended_Elastic;
deltaFz_Lateral_Rear = deltaFz_LRU + ...
    deltaFz_Lateral_Rear_Suspended_Geometric + ...
    deltaFz_Lateral_Rear_Suspended_Elastic;
%F_aero = 0.5 .* Parameter.air_density .* cross_Sectional_Area .* longVel .^2 .* Downforce_Coef;
