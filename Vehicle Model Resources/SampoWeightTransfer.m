function [dFz] = SampoWeightTransfer( Pf, tw_front, tw_rear, sprung_mass,...
    instant_center_height_front, instant_center_height_rear, front_unsprung_CG,...
    rear_unsprung_CG, unsprung_mass_front, unsprung_mass_rear, latAcc) 
%% Test case
if nargin == 0
    
    Pf = ;
    tw_front =;
    tw_rear = ;
    sprung_mass = ;
    instant_center_height_front = ;
    instant_center_height_front = ;
    front_unsprung_CG = ;
    rear_unsprung_CG = ;
    unsprung_mass_front = ;
    unsprung_mass_rear = ;
    latAcc = ;

    return
end

%% Weight Transfer  
    dFz(1) = ( Pf .* Parameter.Susp.ds .* sprung_mass + ...
        instant_center_height_front .* Pf .* sprung_mass + ...
        front_unsprung_CG .* unsprung_mass_front ) .* ...
        latAcc ./ tw_front;

    dFz(2) = ( (1-Parameter.Susp.Pf) .* Parameter.Susp.ds .* sprung_mass + ...
        instant_center_height_rear .* (1-Pf) .* sprung_mass + ...
        rear_unsprung_CG .* unsprung_mass_rear ) .* ...
        latAcc ./ tw_rear;