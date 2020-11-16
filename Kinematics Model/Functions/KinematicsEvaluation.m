function [Rake, Track, Caster, Camber, Toe, InstantCenter, RollCenter, KPI, Scrub] = ...
    KinematicsEvaluation( Design, Attitude, Target, Transform )
%% Optimization Problem Setup & Execution
if strcmp(Target.Axle, 'Front')
    L = Target.Wheelbase;
else
    L = -Target.Wheelbase;
end

x0 = [ L, Target.Track, Target.Camber,  Attitude.Steer, Target.Camber; ...
       L, Target.Track, Target.Camber, -Attitude.Steer, Target.Camber];

lb = repmat( [-Inf, -Inf, -60, -60, 2], 2, 1 );
ub = repmat( [ Inf,  Inf,  60,  60, 10], 2, 1 );

ObjFun = @(x) ObjectiveFunction( x, Design, Attitude, Target, Transform );
[x, fval, exitflag, ~] = fmincon( ObjFun, x0, [], [], [], [], lb, ub, [], optimset( 'Display', 'iter-detailed') );

fval
exitflag
Rake   = x(:,1) + L/2;
Track  = x(:,2);
Camber = x(:,3);
Toe    = x(:,4);
Caster = x(:,5);

%% Evaluate Suspension Metrics
%%% Compute Final World Coordinate Locations
LA(:,1) = Transform.BW( Design.LA, L, Attitude.Ride, ...
    Attitude.Pitch + Target.Rake, Attitude.Roll );
UA(:,1) = Transform.BW( Design.UA, L, Attitude.Ride, ...
    Attitude.Pitch + Target.Rake, Attitude.Roll );
TA(:,1) = Transform.BW( Design.TA + [0 Attitude.Steer 0]', L, Attitude.Ride, ...
    Attitude.Pitch + Target.Rake, Attitude.Roll );

LA(:,2) = Transform.BW( Design.LA, L, Attitude.Ride, ...
    Attitude.Pitch + Target.Rake, -Attitude.Roll );
UA(:,2) = Transform.BW( Design.UA, L, Attitude.Ride, ...
    Attitude.Pitch + Target.Rake, -Attitude.Roll );
TA(:,2) = Transform.BW( Design.TA - [0 Attitude.Steer 0]', L, Attitude.Ride, ...
    Attitude.Pitch + Target.Rake, -Attitude.Roll );

LB(:,1) = Transform.TW( Design.LB, x(1,4), x(1,3), x(1,5), x(1,1), x(1,2), Target.Re);
UB(:,1) = Transform.TW( Design.UB, x(1,4), x(1,3), x(1,5), x(1,1), x(1,2), Target.Re);
TB(:,1) = Transform.TW( Design.TB, x(1,4), x(1,3), x(1,5), x(1,1), x(1,2), Target.Re);

LB(:,2) = Transform.TW( Design.LB, x(2,4), x(2,3), x(2,5), x(2,1), x(2,2), Target.Re);
UB(:,2) = Transform.TW( Design.UB, x(2,4), x(2,3), x(2,5), x(2,1), x(2,2), Target.Re);
TB(:,2) = Transform.TW( Design.TB, x(2,4), x(2,3), x(2,5), x(2,1), x(2,2), Target.Re);

figure
plot3( LA(1,:), LA(2,:), LA(3,:), 'bd', ...
      UA(1,:), UA(2,:), UA(3,:), 'rd', ...
      TA(1,:), TA(2,:), TA(3,:), 'gd', ...
      LB(1,:), LB(2,:), LB(3,:), 'bs', ...
      UB(1,:), UB(2,:), UB(3,:), 'rs', ...
      TB(1,:), TB(2,:), TB(3,:), 'gs' )
axis equal
close

%%% Compute Roll Geometry Metrics
for i = 1:2
    t(i) = det( [LA(2,i)-UA(2,i), UA(2,i)-UB(2,i); LA(3,i)-UA(3,i), UA(3,i)-UB(3,i)] ) ./ ...
           det( [LA(2,i)-LB(2,i), UA(2,i)-UB(2,i); LA(3,i)-LB(3,i), UA(3,i)-UB(3,i)] );
end

InstantCenter = zeros(2,3);

InstantCenter(:,2) = LA(2,:) + t.*( LB(2,:)-LA(2,:) );
InstantCenter(:,3) = LA(3,:) + t.*( LB(3,:)-LA(3,:) );

RollCenter = InstantCenter(:,3) ./ (InstantCenter(:,2) - Target.Track/2) .* (-Target.Track/2);

%%% Compute Wheel Package Metrics
KPI(:,1)   = -atand( (UB(2,:) - LB(2,:)) / (UB(3,:) - LB(3,:)) );
Scrub(:,1) = Target.Track/2 - ( (UB(2,:) - LB(2,:)) / (UB(3,:) - LB(3,:)) * (-LB(3,:)) + LB(2,:) );

%%% Compute Effort Modulus
% <<Work Needed>>

%%% Residual
FVal(1,1) = abs( norm( LA(:,1) - LB(:,1) ) - Design.Length.LA );
FVal(1,2) = abs( norm( UA(:,1) - UB(:,1) ) - Design.Length.UA );
FVal(1,3) = abs( norm( TA(:,1) - TB(:,1) ) - Design.Length.TR );
FVal(1,4) = abs(     ( LA(3,1) - LB(3,1) ) .* tand( Attitude.Pitch ) + LB(1,1) - LA(1,1) );
FVal(1,5) = abs(     ( UA(3,1) - UB(3,1) ) .* tand( Attitude.Pitch ) + UB(1,1) - UA(1,1) );

FVal(2,1) = abs( norm( LA(:,2) - LB(:,2) ) - Design.Length.LA );
FVal(2,2) = abs( norm( UA(:,2) - UB(:,2) ) - Design.Length.UA );
FVal(2,3) = abs( norm( TA(:,2) - TB(:,2) ) - Design.Length.TR );
FVal(2,4) = abs(     ( LA(3,2) - LB(3,2) ) .* tand( Attitude.Pitch ) + LB(1,2) - LA(1,2) );
FVal(2,5) = abs(     ( UA(3,2) - UB(3,2) ) .* tand( Attitude.Pitch ) + UB(1,2) - UA(1,2) );
   
a = 1;

%% Objective Function
    function Fval = ObjectiveFunction( x, Design, Attitude, Target, Transform )
        %%% Inboard Hard Point Transforms
        WCS.LA(:,1) = Transform.BW( Design.LA, L, Attitude.Ride, ...
            Attitude.Pitch + Target.Rake, Attitude.Roll );
        WCS.UA(:,1) = Transform.BW( Design.UA, L, Attitude.Ride, ...
            Attitude.Pitch + Target.Rake, Attitude.Roll );
        WCS.TA(:,1) = Transform.BW( Design.TA + [0 Attitude.Steer 0]', L, Attitude.Ride, ...
            Attitude.Pitch + Target.Rake, Attitude.Roll );

        WCS.LA(:,2) = Transform.BW( Design.LA, L, Attitude.Ride, ...
            Attitude.Pitch + Target.Rake, -Attitude.Roll );
        WCS.UA(:,2) = Transform.BW( Design.UA, L, Attitude.Ride, ...
            Attitude.Pitch + Target.Rake, -Attitude.Roll );
        WCS.TA(:,2) = Transform.BW( Design.TA + [0 -Attitude.Steer 0]', L, Attitude.Ride, ...
            Attitude.Pitch + Target.Rake, -Attitude.Roll );

        WCS.LB(:,1) = Transform.TW( Design.LB, x(1,4), x(1,3), ...
            Attitude.Pitch + Target.Caster, x(1,1), x(1,2), Target.Re);
        WCS.UB(:,1) = Transform.TW( Design.UB, x(1,4), x(1,3), ...
            Attitude.Pitch + Target.Caster, x(1,1), x(1,2), Target.Re);
        WCS.TB(:,1) = Transform.TW( Design.TB, x(1,4), x(1,3), ...
            Attitude.Pitch + Target.Caster, x(1,1), x(1,2), Target.Re);

        WCS.LB(:,2) = Transform.TW( Design.LB, x(2,4), x(2,3), ...
            Attitude.Pitch + Target.Caster, x(2,1), x(2,2), Target.Re);
        WCS.UB(:,2) = Transform.TW( Design.UB, x(2,4), x(2,3), ...
            Attitude.Pitch + Target.Caster, x(2,1), x(2,2), Target.Re);
        WCS.TB(:,2) = Transform.TW( Design.TB, x(2,4), x(2,3), ...
            Attitude.Pitch + Target.Caster, x(2,1), x(2,2), Target.Re);
        
        Fval(1,1) = abs( norm( WCS.LA(:,1) - WCS.LB(:,1) ) - Design.Length.LA );
        Fval(1,2) = abs( norm( WCS.UA(:,1) - WCS.UB(:,1) ) - Design.Length.UA );
        Fval(1,3) = abs( norm( WCS.TA(:,1) - WCS.TB(:,1) ) - Design.Length.TR );
        Fval(1,4) = 10*abs(     ( WCS.LA(3,1) - WCS.LB(3,1) ) .* tand( Attitude.Pitch ) + WCS.LB(1,1) - WCS.LA(1,1) );
        Fval(1,5) = 10*abs(     ( WCS.UA(3,1) - WCS.UB(3,1) ) .* tand( Attitude.Pitch ) + WCS.UB(1,1) - WCS.UA(1,1) );

        Fval(2,1) = abs( norm( WCS.LA(:,2) - WCS.LB(:,2) ) - Design.Length.LA );
        Fval(2,2) = abs( norm( WCS.UA(:,2) - WCS.UB(:,2) ) - Design.Length.UA );
        Fval(2,3) = abs( norm( WCS.TA(:,2) - WCS.TB(:,2) ) - Design.Length.TR );
        Fval(2,4) = 10*abs(     ( WCS.LA(3,2) - WCS.LB(3,2) ) .* tand( Attitude.Pitch ) + WCS.LB(1,2) - WCS.LA(1,2) );
        Fval(2,5) = 10*abs(     ( WCS.UA(3,2) - WCS.UB(3,2) ) .* tand( Attitude.Pitch ) + WCS.UB(1,2) - WCS.UA(1,2) );

        Fval = sum( sum( Fval ) );
    end
end