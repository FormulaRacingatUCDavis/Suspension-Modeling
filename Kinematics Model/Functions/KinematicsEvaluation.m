function [Base, Track, Steer, Camber, InstantCenter, RollCenter, Modulus] = ...
    KinematicsEvaluation( Design, Attitude, Target, beta0 )
%% Define Coordinate Transformations
if strcmp(Target.Axle, 'Front')
    L = Target.Wheelbase;
else
    L = -Target.Wheelbase;
end

RotX = @(t) [1       0       0      ;  0       cosd(t) sind(t);  0       -sind(t) cosd(t)];
RotY = @(t) [cosd(t) 0       sind(t);  0       1       0      ; -sind(t)  0       cosd(t)];
RotZ = @(t) [cosd(t) sind(t) 0      ; -sind(t) cosd(t) 0      ;  0        0       1      ];

bTw = @(pW,phi,theta) RotX(phi)    * RotY(theta) * (pW - Target.CG' - [L/2 0 0]');
wTb = @(pB,phi,theta) RotY(theta)' * RotX(phi)'  *  pB + Target.CG' - [L/2 0 0]';

tTw = @(pW,xT,yT,gamma,phi,delta) RotZ(delta) * RotX(gamma)  * RotY(phi)    * (pW - [xT yT Target.Rl]');
wTt = @(pT,xT,yT,gamma,phi,delta) RotY(phi)'  * RotX(gamma)' * RotZ(delta)' *  pT + [xT yT Target.Rl]' ;

laTb = @(pB,beta) RotX(beta)            * RotZ(Design.a.LA(2))  * RotY(Design.a.LA(1)) * (pB - (Design.p.LAb + [L/2 0 Target.Ride-Target.CG(3)]'));
bTla = @(pA,beta) RotY(Design.a.LA(1))' * RotZ(Design.a.LA(2))' * RotX(beta)'          *  pA +  Design.p.LAb + [L/2 0 Target.Ride-Target.CG(3)]'  ;

uaTb = @(pB,beta) RotX(beta)            * RotZ(Design.a.UA(2))  * RotY(Design.a.UA(1)) * (pB - (Design.p.UAb + [L/2 0 Target.Ride-Target.CG(3)]'));
bTua = @(pA,beta) RotY(Design.a.UA(1))' * RotZ(Design.a.UA(2))' * RotX(beta)'          *  pA +  Design.p.UAb + [L/2 0 Target.Ride-Target.CG(3)]'  ;

lbTla = @(pA ,beta1,beta2,beta3) RotZ(beta3)  * RotX(beta1)  * RotY(beta2)  * (pA  - [0 Design.L.LA 0]');
laTlb = @(pLB,beta1,beta2,beta3) RotY(beta2)' * RotX(beta1)' * RotZ(beta3)' *  pLB + [0 Design.L.LA 0]' ;

lbTt = @(pT ) pT  - Design.p.LBt;
tTlb = @(pLB) pLB + Design.p.LBt;

trTb = @(pB ,beta1,beta2) RotZ(beta2)  * RotX(beta1)  * (pB  - (Design.p.TAb + [L/2 Attitude.Steer Target.Ride-Target.CG(3)]'));
bTtr = @(pTR,beta1,beta2) RotX(beta1)' * RotZ(beta2)' *  pTR +  Design.p.TAb + [L/2 Attitude.Steer Target.Ride-Target.CG(3)]'  ;

%% Optimization Problem Setup & Execution
if nargin < 4
    beta0 = [ 10, ... % Lower A-Arm Rotation
              10, ... % Upper A-Arm Rotation
              10, -10, ... % Tie Rod (x,z)-Axis Rotations
              3, 0, 0]; % Lower Ball Joint (x,y,z)-Axis Rotations
end

lb = repmat( [-10 -20 -20 -30 -10 -10 -40], 2, 1 );
ub = repmat( [ 30  35  30  30  10  10  40], 2, 1 );

ObjFun = @(beta) ObjectiveFunction( beta, Design, Attitude, Target );
[x, fval, exitflag, ~] = fmincon( ObjFun, beta0, [], [], [], [], lb, ub, [], optimset( 'Display', 'iter-detailed') );

x;

%% Objective Function
    function Fval = ObjectiveFunction( beta, Design, Attitude, Target )
        UB = bTla( laTlb( lbTt(Design.p.UBt), beta(5), beta(6), beta(7) ), beta(1) ) - ...
             bTua( [0 Design.L.UA 0]', beta(2) );
        
        TB = bTla( laTlb( lbTt(Design.p.TBt), beta(5), beta(6), beta(7) ), beta(1) ) - ...
             bTtr( [0 Design.L.TR 0]', beta(3), beta(4) );
        
        oT = wTb( bTla( laTlb( -Design.p.LBt, beta(5), beta(6), beta(7) ), beta(1) ), Attitude.Roll, Attitude.Pitch );
         
        Rl = oT(3) - Target.Rl;
        
        Fval = norm(UB) + norm(TB) + Rl.^2;
    end

    function Fval = SuspensionMetrics( beta, Design, Attitude, Target )
        %% World Coordinates
        LA = wTb( bTla( [0 0 0]', beta(1) ), Attitude.Roll, Attitude.Pitch );
        UA = wTb( bTua( [0 0 0]', beta(2) ), Attitude.Roll, Attitude.Pitch );
        TA = wTb( bTtr( [0 0 0]', beta(3), beta(4) ), Attitude.Roll, Attitude.Pitch );
        
        LB = wTb( bTla( [0 Design.L.LA 0]', beta(1) ), Attitude.Roll, Attitude.Pitch );
        UB = wTb( bTua( [0 Design.L.UA 0]', beta(2) ), Attitude.Roll, Attitude.Pitch );
        TB = wTb( bTtr( [0 Design.L.TR 0]', beta(3), beta(4) ), Attitude.Roll, Attitude.Pitch );
        
        T = wTb( bTla( laTlb( -Design.p.LBt, beta(5), beta(6), beta(7) ), beta(1) ), Attitude.Roll, Attitude.Pitch );
                
        %% Wheel Position & Orientation
        Base = T(1);
        Track = T(2);
        
        
    end

    
end