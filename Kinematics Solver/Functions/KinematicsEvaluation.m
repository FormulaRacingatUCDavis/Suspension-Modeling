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

%bTe = @(pW,phi,theta) RotX(phi)    * RotY(theta) * (pW - Target.CG' - [L/2 0 0]'); %world to body
%eTb = @(pB,phi,theta) RotY(theta)' * RotX(phi)'  *  pB + [-L/2 0 -Target.Rl]'; %body to world (old)
eTb = @(p,L,Re,phi,theta) RotX(phi) * RotY(theta) * p + [L/2 0 Re]'; %body to world(earth)

%tTe = @(pW,xT,yT,gamma,phi,delta) RotZ(delta) * RotX(gamma)  * RotY(phi) * (pW - [xT yT Target.Rl]'); %world to tire
%eTt = @(pT,xT,yT,gamma,phi,delta) RotY(phi)'  * RotX(gamma)' * RotZ(delta)' *  pT + [xT yT Target.Rl]' ; %old copy - tire to world
%eTt = @(pT,xT,yT,gamma,delta) RotX(gamma)' * RotZ(delta)' *  pT + [xT yT 0]' ; %tire to world

%tTe = @(pW,xT,yT,gamma,phi,delta) RotZ(delta) * RotX(gamma)  * RotY(phi) * (pW - [xT yT Target.Rl]');
%eTt = @(pT,xT,yT,gamma,phi,delta) RotY(phi)'  * RotX(gamma)' * RotZ(delta)' *  pT + [xT yT Target.Rl]' ;

%whTt = @
tTw = @(pWH, Re, camber, caster) RotX(camber) * RotY(caster)* pWH + [0 0 Re]'; %wheel to tire
%tbTwh = @ (pWH) pWH - Design.tr;

%laTb = @(pB,beta) RotX(beta) * RotZ(Design.a.LA(2)) * RotY(Design.a.LA(1)) * (pB - Design.p.LAb); %body to lower a-arm
bTla = @(pA,beta) RotY(Design.a.UA(1))' * RotZ(Design.a.UA(2))' * RotX(beta)' * pA + Design.p.LAb + [0 0 Attitude.Ride-Target.Ride]'; %lower a-arm to body

%uaTb = @(pB,beta) RotX(beta) * RotZ(Design.a.UA(2)) * RotY(Design.a.UA(1)) * (pB - Design.p.UAb); %body to upper a-arm
bTua = @(pA,beta) RotY(Design.a.UA(1))' * RotZ(Design.a.UA(2))' * RotX(beta)' * pA + Design.p.UAb + [0 0 Attitude.Ride-Target.Ride]'; %upper a-arm to body

%lbTla = @(pA,beta1,beta2,beta3) RotZ(beta3) * RotX(beta1) * RotY(beta2) * (pA  - [0 Design.L.LA 0]'); %lower a-arm to lower ball joint
laTlb = @(pLB,beta1,beta2,beta3) RotY(beta2)' * RotX(beta1)' * RotZ(beta3)' *  pLB + [0 Design.L.LA 0]' ; %lower ball joint to lower a-arm

lbTw = @(pT) pT - Design.p.LBt; %wheel to lower ball joint
%wTlb = @(pLB) pLB + Design.p.LBt; %lower ball joint to wheel

% trTb = @(pB ,beta1,beta2) RotZ(beta2)  * RotX(beta1)  * (pB  - (Design.p.TAb + [0 Attitude.Steer 0]')); %body to tie rod
bTtr = @(pTR,beta1,beta2) RotX(beta1)' * RotZ(beta2)' * pTR + Design.p.TAb + [0 Attitude.Steer Attitude.Ride-Target.Ride]'; %tie rod to body


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
    SuspensionMetrics( beta, Attitude, Target );

%%% Local Functions
    function Fval = ObjectiveFunction( beta, Design, Attitude, Target )
        UB = bTla( laTlb( lbTw(Design.p.UBt), beta(5), beta(6), beta(7) ), beta(1) ) - ...
             bTua( [0 Design.L.UA 0]', beta(2) );
        
        TB = bTla( laTlb( lbTw(Design.p.TBt), beta(5), beta(6), beta(7) ), beta(1) ) - ...
             bTtr( [0 Design.L.TR 0]', beta(3), beta(4) );
        
        oT = eTb( bTla( laTlb( -Design.p.LBt, beta(5), beta(6), beta(7) ), beta(1) ), L, Target.Rl, Attitude.Roll, Attitude.Pitch );
         
        Rl = oT(3) - Target.Rl;
        
        Fval = norm(UB) + norm(TB) + Rl.^2;
    end

    function [Base, Track, Steer, Camber, InstantCenter, RollCenter, Modulus] = ...
            SuspensionMetrics( beta, Attitude, Target )       
        %%% Wheel Position & Orientation
        % Wheel Position
        T = eTb( bTla( laTlb( lbTw([0 0 0]'), beta(5), beta(6), beta(7) ), beta(1) ), L, Target.Rl, Attitude.Roll, Attitude.Pitch );
        % T = wTt([0 0 0]', Steer, Camber)
        Base = T(1);
        Track = T(2);
         
        % Solve for Tire Unit Vectors
        xT = eTb( bTla( laTlb( lbTw([1 0 0]'), beta(5), beta(6), beta(7) ), beta(1) ), L, Target.Rl, Attitude.Roll, Attitude.Pitch );
        % xT = wTt([1 0 0]', Steer, Camber)
        yT = eTb( bTla( laTlb( lbTw([0 1 0]'), beta(5), beta(6), beta(7) ), beta(1) ), L, Target.Rl, Attitude.Roll, Attitude.Pitch );
        % yT = wTt([0 1 0]', Steer, Camber)
        zT = eTb( bTla( laTlb( lbTw([0 0 1]'), beta(5), beta(6), beta(7) ), beta(1) ), L, Target.Rl, Attitude.Roll, Attitude.Pitch );
        % zT = wTt([0 0 1]', Steer, Camber)
        
        xT = xT - T;
        yT = yT - T;
        zT = zT - T;
         
        % Solve for Tire Positions
        posT = eTb( bTla( laTlb( lbTw( tTw([0 0 0]', Target.Camber, Target.KPI, Target.Caster) ), beta(5), beta(6), beta(7) ),...
            beta(1) ), L, Target.Rl, Attitude.Roll, Attitude.Pitch );
        
        % Solve for Steer & Camber [Sign Issues]
        Camber = acosd( dot( yT, [0 0 1]' ) ) - 90; % Camber Angle [deg]

        k = cross( zT, [0 0 1]' ) ./ sind(Camber); % Camber Rotation Vector
        
        yTa = yT*cosd(Camber) + cross(k,yT)*sind(Camber) + ...
            k*dot(k,yT)*(1-cosd(Camber)); % Ground Parallel Longitudinal Tire Axis
        yTa = yTa ./ norm(yTa);
        
        Steer = -sign(yTa(1)) * acosd( dot( yTa, [0 1 0]' ) ); % Steer Angle [deg]
        
        %%% Roll Geometry Metrics
        pLA = eTb( bTla( [0 0 0]', beta(1) ), L, Target.Rl, Attitude.Roll, Attitude.Pitch );
        pUA = eTb( bTua( [0 0 0]', beta(2) ), L, Target.Rl, Attitude.Roll, Attitude.Pitch );
        pTA = eTb( bTtr( [0 0 0]', beta(3), beta(4) ), L, Target.Rl, Attitude.Roll, Attitude.Pitch );
        
        pLB = eTb( bTla( [0 Design.L.LA 0]', beta(1) ), L, Target.Rl, Attitude.Roll, Attitude.Pitch );
        pUB = eTb( bTua( [0 Design.L.UA 0]', beta(2) ), L, Target.Rl, Attitude.Roll, Attitude.Pitch );
        pTB = eTb( bTtr( [0 Design.L.TR 0]', beta(3), beta(4) ), L, Target.Rl, Attitude.Roll, Attitude.Pitch );
               
        InstantCenter = LineIntersection( pLA(2:3), pLB(2:3), pUA(2:3), pUB(2:3) );
        RollCenter = LineIntersection( InstantCenter, [Track,0], [0,0], [0,1] );
        
        %%% KPI / Scrub Calculator
        
        %%% Steering Effort
%         % Solve for Tire Forces
%         Tire = Target.TireParameter.Tire.Pacejka;
%         SlipAngle   = Steer;         %[degrees]
%         SlipRatio   = 0.00;             %[ ] Assume no slip
%         NormalLoad  = 270 * 9.81/4;  %[N] About 1/4 the weight of the car
%         Pressure    = 70;            %[kPa] 10 psi
%         Inclination = Target.Camber; %[degrees]
%         Velocity    = 10;            %[m/s]
%         Idx         = 1;             %[ ]
%         Model       = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );
%         
%         [Fx, Fy, Mz, ~, ~] = ContactPatchLoads( Tire, SlipAngle, ...
%             SlipRatio, NormalLoad, Pressure, Inclination, Velocity, Idx, Model );
%         
%         % Solve for Moments from tire and tie rod
%         ToeBase = T-pTB; %distance from tie rod pickup point to center of wheel
%         disT = posT - T; %posT is contact patch, T is center of wheel
%         dirTR = ( pTB-pTA ) ./ ( norm( pTB-pTA ) ); %direction of the force in the tie rod
%         
%         %Solve for Modulus
%         TireForce = [Fx, Fy, Mz]; %tire forces matrix from tire model
%         TireDirection = [ xT; yT ]; %tire direction matrix
%         Modulus = zeros( size(TireForce) ); %initiailize modulus matrix
%         
%         for i = size(TireForce)
%             if i <= size(TireDirection,1)
%                 ModulusMoment = cross( disT, TireDirection(i,:) ); %Tire Moment
%             else
%                 ModulusMoment = [ 0, 0, 1 ]; %Tire Moment
%             end
%             Modulus(i) = [0, 0, ModulusMoment(3)] ./ (cross( ToeBase, dirTR) );
%         end
%         
%         %Solve for Steering Effort
%         TieRodForce = TireForce.*Modulus'; %Solves force into the tie rod
%         SteeringEffort = TieRodForce(2)*( 0.08788 / ( 2*pi ) ); %torque at the steering wheel needed to turn wheel [N*m] (rack travel 87.88mm/deg)
        Modulus = 0;
        
        %%% Debugging
        %{ 
        plot3( LA(1), LA(2), LA(3), 'bx', ...
               UA(1), UA(2), UA(3), 'rx', ...
               TA(1), TA(2), TA(3), 'gx', ...
               LB(1), LB(2), LB(3), 'b+', ...
               UB(1), UB(2), UB(3), 'r+', ...
               TB(1), TB(2), TB(3), 'g+', ...
                T(1),  T(2),  T(3), 'mx');
        quiver3( 0, 0, 0, yT(1), yT(2), yT(3) ); hold on;
        %}
    end

    function pX = LineIntersection(p1, p2, p3, p4)
        pX(1) = det( [det( [p1(1) p1(2); p2(1) p2(2)] ), det( [p1(1) 1; p2(1) 1] ); ...
                      det( [p3(1) p3(2); p4(1) p4(2)] ), det( [p3(1) 1; p4(1) 1] )] ) ./ ...
                det( [det( [p1(1) 1    ; p2(1) 1    ] ), det( [p1(2) 1; p2(2) 1] ); ...
                      det( [p3(1) 1    ; p4(1) 1    ] ), det( [p3(2) 1; p4(2) 1] )] ); 
                  
        pX(2) = det( [det( [p1(1) p1(2); p2(1) p2(2)] ), det( [p1(2) 1; p2(2) 1] ); ...
                      det( [p3(1) p3(2); p4(1) p4(2)] ), det( [p3(2) 1; p4(2) 1] )] ) ./ ...
                det( [det( [p1(1) 1    ; p2(1) 1    ] ), det( [p1(2) 1; p2(2) 1] ); ...
                      det( [p3(1) 1    ; p4(1) 1    ] ), det( [p3(2) 1; p4(2) 1] )] );
    end
end