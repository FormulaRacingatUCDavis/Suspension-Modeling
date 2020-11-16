function [Rake, Track, Caster, Camber, Toe, InstantCenter, RollCenter, KPI, Scrub] = ...
    KinematicsEvaluation2( Design, Attitude, Target, beta0 )
%% Define Coordinate Transformations
if strcmp(Target.Axle, 'Front')
    L = Target.Wheelbase;
else
    L = -Target.Wheelbase;
end

RotX = @(t) [1       0       0      ;  0       cosd(t) sind(t);  0       -sind(t) cosd(t)];
RotY = @(t) [cosd(t) 0       sind(t);  0       1       0      ; -sind(t)  0       cosd(t)];
RotZ = @(t) [cosd(t) sind(t) 0      ; -sind(t) cosd(t) 0      ;  0        0       1      ];

wTb = @(phi,theta) RotX(phi) * RotY(theta) * pW + Target.CG';
bTw = @(phi,theta) (RotX(phi) * RotY(theta)) \ (pB - Target.CG');

wTt = @(pW,xT,yT,gamma,phi,delta) RotZ(delta) * RotX(gamma) * RotY(phi) * pW + [xT yT Target.Rl]';
tTw = @(pT,xT,yT,zT,gamma,phi,delta) (RotZ(delta) * RotX(gamma) * RotY(phi)) \ (pT - [xT yT Target.Rl]');

bTla = @(pB,beta) RotX(beta) * RotZ(Design.a.LA(2)) * RotY(Design.a.LA(1)) * pB + Design.p.LAb';
laTb = @(pA,beta) (RotX(beta) * RotZ(Design.a.LA(2)) * RotY(Design.a.LA(1))) \ (pA - Design.p.LAb');

bTua = @(pB,beta) RotX(beta) * RotZ(Design.a.UA(2)) * RotY(Design.a.UA(1)) * pB + Design.p.UAb';
uaTb = @(pA,beta) (RotX(beta) * RotZ(Design.a.UA(2)) * RotY(Design.a.UA(1))) \ (pA - Design.p.UAb');

laTlb = @(pA,beta1,beta2,beta3) RotZ(beta3) * RotX(beta1) * RotY(beta2) * pA + [0 Design.L.LA 0]';
lbTla = @(pLB,beta1,beta2,beta3) (RotZ(beta3) * RotX(beta1) * RotY(beta2)) \ (pLB - [0 Design.L.LA 0]');

lbTt = @(pLB) pLB - Design.p.LBt';
tTlb = @(pT)  pT  + Design.p.LBt';

bTtr = @(pB,beta1,beta2) RotZ(beta2) * RotX(beta1) * pTR + Design.p.TAb';
trTb = @(pTR,beta1,beta2) (RotZ(beta2) * RotX(beta1)) \ (pTR - Design.p.TAb');

%% Optimization Problem Setup & Execution
if nargin < 4
    beta0 = [ 15, ... % Lower A-Arm Rotation
              10, ... % Upper A-Arm Rotation
              10, -10, ... % Tie Rod (x,z)-Axis Rotations
              3, 0, 0]; % Lower Ball Joint (x,y,z)-Axis Rotations
end

lb = repmat( [-10 -20 -20 -30 -10 -10 -40], 2, 1 );
ub = repmat( [ 30  35  30  30  10  10  40], 2, 1 );

ObjFun = @(beta) ObjectiveFunction( beta, Design, Attitude, Target );
[x, fval, exitflag, ~] = fmincon( ObjFun, beta0, [], [], [], [], lb, ub, [], optimset( 'Display', 'iter-detailed') );

%{
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
%}

%% Objective Function
    function Fval = ObjectiveFunction( beta, Design, Attitude, Target )
        
    end
end