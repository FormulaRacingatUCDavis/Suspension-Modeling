function [dFz] = SampoWeightTransfer( Pf, tw_front, tw_rear, ds, sprung_mass,...
    instant_center_height_front, instant_center_height_rear, front_unsprung_CG,...
    rear_unsprung_CG, unsprung_mass_front, unsprung_mass_rear, latAcc) 
%% Test case
if nargin == 0
    
    Pf = 0.50;
    tw_front = 1.22;
    tw_rear = 1.22;
    sprung_mass = 191;
    instant_center_height_front = 0.025;
    instant_center_height_rear = 0.05;
    front_unsprung_CG = 0.2;
    rear_unsprung_CG = 0.3;
    unsprung_mass_front = 38.5;
    unsprung_mass_rear = 38.5;
    latAcc = 3.0;
    sprung_center_mass_height = 0.23;
    wheelbase = 1.525;
    ds = 0;
    
    ds = sprung_center_mass_height  - ...
    interp1( [0 wheelbase], [instant_center_height_front instant_center_height_rear], ...
    Pf.*wheelbase );
    
    [dFz] = SampoWeightTransfer( Pf, tw_front, tw_rear, ds, sprung_mass,...
    instant_center_height_front, instant_center_height_rear, front_unsprung_CG,...
    rear_unsprung_CG, unsprung_mass_front, unsprung_mass_rear, latAcc)

    return

end

%% Weight Transfer  
    dFz(1) = ( Pf .* ds .* sprung_mass + ...
        instant_center_height_front .* Pf .* sprung_mass + ...
        front_unsprung_CG .* unsprung_mass_front ) .* ...
        latAcc ./ tw_front;

    dFz(2) = ( (1-Pf) .* ds .* sprung_mass + ...
        instant_center_height_rear .* (1-Pf) .* sprung_mass + ...
        rear_unsprung_CG .* unsprung_mass_rear ) .* ...
        latAcc ./ tw_rear;