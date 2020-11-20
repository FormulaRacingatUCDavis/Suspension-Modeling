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

% bTw = @(pW,phi,theta) RotX(phi)    * RotY(theta) * (pW - Target.CG' - [L/2 0 0]');
wTb = @(pB,phi,theta) RotY(theta)' * RotX(phi)'  *  pB + Target.CG' + [-L/2 0 Attitude.Ride-50.8]';

% tTw = @(pW,xT,yT,gamma,phi,delta) RotZ(delta) * RotX(gamma)  * RotY(phi)    * (pW - [xT yT Target.Rl]');
% wTt = @(pT,xT,yT,gamma,phi,delta) RotY(phi)'  * RotX(gamma)' * RotZ(delta)' *  pT + [xT yT Target.Rl]' ;

% laTb = @(pB,beta) RotX(beta)            * RotZ(Design.a.LA(2))  * RotY(Design.a.LA(1)) * (pB - (Design.p.LAb + [L/2 0 Target.Ride-Target.CG(3)]'));
bTla = @(pA,beta) RotY(Design.a.LA(1))' * RotZ(Design.a.LA(2))' * RotX(beta)'          *  pA +  Design.p.LAb + [L/2 0 Target.Ride-Target.CG(3)]'  ;

% uaTb = @(pB,beta) RotX(beta)            * RotZ(Design.a.UA(2))  * RotY(Design.a.UA(1)) * (pB - (Design.p.UAb + [L/2 0 Target.Ride-Target.CG(3)]'));
bTua = @(pA,beta) RotY(Design.a.UA(1))' * RotZ(Design.a.UA(2))' * RotX(beta)'          *  pA +  Design.p.UAb + [L/2 0 Target.Ride-Target.CG(3)]'  ;

% lbTla = @(pA ,beta1,beta2,beta3) RotZ(beta3)  * RotX(beta1)  * RotY(beta2)  * (pA  - [0 Design.L.LA 0]');
laTlb = @(pLB,beta1,beta2,beta3) RotY(beta2)' * RotX(beta1)' * RotZ(beta3)' *  pLB + [0 Design.L.LA 0]' ;

lbTt = @(pT ) pT  - Design.p.LBt;
% tTlb = @(pLB) pLB + Design.p.LBt;

% trTb = @(pB ,beta1,beta2) RotZ(beta2)  * RotX(beta1)  * (pB  - (Design.p.TAb + [L/2 Attitude.Steer Target.Ride-Target.CG(3)]'));
bTtr = @(pTR,beta1,beta2) RotX(beta1)' * RotZ(beta2)' *  pTR +  Design.p.TAb + [L/2 Attitude.Steer Target.Ride-Target.CG(3)]'  ;

%% Optimization Problem Setup & Execution
if nargin < 4
    beta0 = [ 1, ... % Lower A-Arm Rotation
              5, ... % Upper A-Arm Rotation
              0, 0, ... % Tie Rod (x,z)-Axis Rotations
              0, 0, 0]; % Lower Ball Joint (x,y,z)-Axis Rotations
end

lb = [-10 -20 -20 -30 -10 -10 -40];
ub = [ 30  35  30  30  10  10  40];

ObjFun = @(beta) ObjectiveFunction( beta, Design, Attitude, Target );
[beta, fval] = fmincon( ObjFun, beta0, [], [], [], [], lb, ub, [], optimset( 'Display', 'off' ) );

[Base, Track, Steer, Camber, InstantCenter, RollCenter, Modulus] = ...
    SuspensionMetrics( beta, Attitude );

%%% Local Functions
    function Fval = ObjectiveFunction( beta, Design, Attitude, Target )
        UB = bTla( laTlb( lbTt(Design.p.UBt), beta(5), beta(6), beta(7) ), beta(1) ) - ...
             bTua( [0 Design.L.UA 0]', beta(2) );
        
        TB = bTla( laTlb( lbTt(Design.p.TBt), beta(5), beta(6), beta(7) ), beta(1) ) - ...
             bTtr( [0 Design.L.TR 0]', beta(3), beta(4) );
        
        oT = wTb( bTla( laTlb( -Design.p.LBt, beta(5), beta(6), beta(7) ), beta(1) ), Attitude.Roll, Attitude.Pitch );
         
        Rl = oT(3) - Target.Rl;
        
        Fval = norm(UB) + norm(TB) + Rl.^2;
    end

    function [Base, Track, Steer, Camber, InstantCenter, RollCenter, Modulus] = ...
            SuspensionMetrics( beta, Attitude )   
        %%% Wheel Position & Orientation
        % Tire Position
        T = wTb( bTla( laTlb( lbTt([0 0 0]'), beta(5), beta(6), beta(7) ), beta(1) ), Attitude.Roll, Attitude.Pitch );
        
        Base = T(1);
        Track = T(2);
         
        % Solve for Tire Unit Vectors
        xT = wTb( bTla( laTlb( lbTt([1 0 0]'), beta(5), beta(6), beta(7) ), beta(1) ), Attitude.Roll, Attitude.Pitch );
        yT = wTb( bTla( laTlb( lbTt([0 1 0]'), beta(5), beta(6), beta(7) ), beta(1) ), Attitude.Roll, Attitude.Pitch );
        zT = wTb( bTla( laTlb( lbTt([0 0 1]'), beta(5), beta(6), beta(7) ), beta(1) ), Attitude.Roll, Attitude.Pitch );
        
        xT = xT - T;
        yT = yT - T;
        zT = zT - T;
         
        % Solve for Steer & Camber [Sign Issues]
        Camber = acosd( dot( yT, [0 0 1]' ) ) - 90; % Camber Angle [deg]

        k = cross( zT, [0 0 1]' ) ./ sind(Camber); % Camber Rotation Vector
        
        yTa = yT*cosd(Camber) + cross(k,yT)*sind(Camber) + ...
            k*dot(k,yT)*(1-cosd(Camber)); % Ground Parallel Longitudinal Tire Axis
        yTa = yTa ./ norm(yTa);
        
        Steer = -sign(yTa(1)) * acosd( dot( yTa, [0 1 0]' ) ); % Steer Angle [deg]
        
        %%% Roll Geometry Metrics
        LA = wTb( bTla( [0 0 0]', beta(1) ), Attitude.Roll, Attitude.Pitch );
        UA = wTb( bTua( [0 0 0]', beta(2) ), Attitude.Roll, Attitude.Pitch );
        TA = wTb( bTtr( [0 0 0]', beta(3), beta(4) ), Attitude.Roll, Attitude.Pitch );
        
        LB = wTb( bTla( [0 Design.L.LA 0]', beta(1) ), Attitude.Roll, Attitude.Pitch );
        UB = wTb( bTua( [0 Design.L.UA 0]', beta(2) ), Attitude.Roll, Attitude.Pitch );
        TB = wTb( bTtr( [0 Design.L.TR 0]', beta(3), beta(4) ), Attitude.Roll, Attitude.Pitch );
        
        %plot3( LA(1), LA(2), LA(3), 'bx', ...
        %       UA(1), UA(2), UA(3), 'rx', ...
        %       TA(1), TA(2), TA(3), 'gx', ...
        %       LB(1), LB(2), LB(3), 'b+', ...
        %       UB(1), UB(2), UB(3), 'r+', ...
        %       TB(1), TB(2), TB(3), 'g+', ...
        %        T(1),  T(2),  T(3), 'mx');
               
        InstantCenter = 0;
        RollCenter = 0;
        
        %%% Steering Effort
        Modulus = 0;  
        
        % quiver3( 0, 0, 0, yT(1), yT(2), yT(3) ); hold on;
    end
end