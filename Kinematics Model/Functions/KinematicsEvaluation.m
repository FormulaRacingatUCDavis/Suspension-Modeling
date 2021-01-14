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

% bTo = @(pW,phi,theta) RotX(phi)    * RotY(theta) * (pW - Target.CG' - [L/2 0 0]');
oTb = @(pB,phi,theta) RotY(theta)' * RotX(phi)'  *  pB + Target.CG' + [-L/2 0 Attitude.Ride-50.8]';

% wTo = @(pW,xT,yT,gamma,phi,delta) RotZ(delta) * RotX(gamma)  * RotY(phi)    * (pW - [xT yT Target.Rl]');
% oTw = @(pT,xT,yT,gamma,phi,delta) RotY(phi)'  * RotX(gamma)' * RotZ(delta)' *  pT + [xT yT Target.Rl]' ;

% laTb = @(pB,beta) RotX(beta)            * RotZ(Design.a.LA(2))  * RotY(Design.a.LA(1)) * (pB - (Design.p.LAb + [L/2 0 Target.Ride-Target.CG(3)]'));
bTla = @(pA,beta) RotY(Design.a.LA(1))' * RotZ(Design.a.LA(2))' * RotX(beta)'          *  pA +  Design.p.LAb + [L/2 0 Target.Ride-Target.CG(3)]'  ;

% uaTb = @(pB,beta) RotX(beta)            * RotZ(Design.a.UA(2))  * RotY(Design.a.UA(1)) * (pB - (Design.p.UAb + [L/2 0 Target.Ride-Target.CG(3)]'));
bTua = @(pA,beta) RotY(Design.a.UA(1))' * RotZ(Design.a.UA(2))' * RotX(beta)'          *  pA +  Design.p.UAb + [L/2 0 Target.Ride-Target.CG(3)]'  ;

% lbTla = @(pA ,beta1,beta2,beta3) RotZ(beta3)  * RotX(beta1)  * RotY(beta2)  * (pA  - [0 Design.L.LA 0]');
laTlb = @(pLB,beta1,beta2,beta3) RotY(beta2)' * RotX(beta1)' * RotZ(beta3)' *  pLB + [0 Design.L.LA 0]' ;

lbTw = @(pT ) pT  - Design.p.LBt;
% wTlb = @(pLB) pLB + Design.p.LBt;\

% trTb = @(pB ,beta1,beta2) RotZ(beta2)  * RotX(beta1)  * (pB  - (Design.p.TAb + [L/2 Attitude.Steer Target.Ride-Target.CG(3)]'));
bTtr = @(pTR,beta1,beta2) RotX(beta1)' * RotZ(beta2)' *  pTR +  Design.p.TAb + [L/2 Attitude.Steer Target.Ride-Target.CG(3)]'  ;

tTw = @(pW,beta1) RotX(beta1) * (pW - [0 0 Target.Rl]);
wTt = @(pT,beta1) RotX(beta1)' * pW + [0 0 Target.Rl];

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
    SuspensionMetrics( beta, Attitude, Target);

%%% Local Functions
    function Fval = ObjectiveFunction( beta, Design, Attitude, Target )
        UB = bTla( laTlb( lbTw(Design.p.UBt), beta(5), beta(6), beta(7) ), beta(1) ) - ...
             bTua( [0 Design.L.UA 0]', beta(2) );
        
        TB = bTla( laTlb( lbTw(Design.p.TBt), beta(5), beta(6), beta(7) ), beta(1) ) - ...
             bTtr( [0 Design.L.TR 0]', beta(3), beta(4) );
        
        oT = oTb( bTla( laTlb( -Design.p.LBt, beta(5), beta(6), beta(7) ), beta(1) ), Attitude.Roll, Attitude.Pitch );
         
        Rl = oT(3) - Target.Rl;
        
        Fval = norm(UB) + norm(TB) + Rl.^2;
    end

    function [Base, Track, Steer, Camber, InstantCenter, RollCenter, Modulus] = ...
            SuspensionMetrics( beta, Attitude, Target )   
        %%% Wheel Position & Orientation
        % Tire Position
        T = oTb( bTla( laTlb( lbTw([0 0 -Target.Rl]'), beta(5), beta(6), beta(7) ), beta(1) ), Attitude.Roll, Attitude.Pitch );
        
        Base = T(1);
        Track = T(2);
         
        % Solve for Tire Unit Vectors
        W = oTb( bTla( laTlb( lbTw([0 0 0]'), beta(5), beta(6), beta(7) ), beta(1) ), Attitude.Roll, Attitude.Pitch );

        xW = oTb( bTla( laTlb( lbTw([1 0 0]'), beta(5), beta(6), beta(7) ), beta(1) ), Attitude.Roll, Attitude.Pitch );
        yW = oTb( bTla( laTlb( lbTw([0 1 0]'), beta(5), beta(6), beta(7) ), beta(1) ), Attitude.Roll, Attitude.Pitch );
        zW = oTb( bTla( laTlb( lbTw([0 0 1]'), beta(5), beta(6), beta(7) ), beta(1) ), Attitude.Roll, Attitude.Pitch );
        
        xW = xW - W;
        yW = yW - W;
        zW = zW - W;
         
        % Solve for Steer & Camber [Sign Issues]
        Camber = acosd( dot( yW, [0 0 1]' ) ) - 90; % Camber Angle [deg]

        k = cross( zW, [0 0 1]' ) ./ sind(Camber); % Camber Rotation Vector Rodrigues
        
        yWa = yW*cosd(Camber) + cross(k,yW)*sind(Camber) + ...
            k*dot(k,yW)*(1-cosd(Camber)); % Ground Parallel Longitudinal Tire Axis
        yWa = yWa ./ norm(yWa);
        
        Steer = -sign(yWa(1)) * acosd( dot( yWa, [0 1 0]' ) ); % Steer Angle [deg]
        
        %%% Roll Geometry Metrics
        pLAo = oTb( bTla( [0 0 0]', beta(1) ), Attitude.Roll, Attitude.Pitch );
        pUAo = oTb( bTua( [0 0 0]', beta(2) ), Attitude.Roll, Attitude.Pitch );
        pTAo = oTb( bTtr( [0 0 0]', beta(3), beta(4) ), Attitude.Roll, Attitude.Pitch );
        
        pLBo = oTb( bTla( [0 Design.L.LA 0]', beta(1) ), Attitude.Roll, Attitude.Pitch );
        pUBo = oTb( bTua( [0 Design.L.UA 0]', beta(2) ), Attitude.Roll, Attitude.Pitch );
        pTBo = oTb( bTtr( [0 Design.L.TR 0]', beta(3), beta(4) ), Attitude.Roll, Attitude.Pitch );
               
        InstantCenter = LineIntersection( pLAo(2:3), pLBo(2:3), pUAo(2:3), pUBo(2:3) );
        RollCenter = LineIntersection( InstantCenter, [Track,0], [0,0], [0,1] );
        
        %%% KPI / Scrub Calculator
        %KPI=acosd(dot([0 0 1],(pUB-pLB))./norm(pUB-pLB)); %KPI in degrees from ground plane normal vector
        Scrub=Track/2-((pLBo./(pLBo-pUBo)).*(pUBo(2)-pLBo(2))+pLBo(2)); %Mechanical scrub radius
        mechTrail=Base-((pLBo(3)./(pLBo(3)-pUBo(3))).*(pUBo(1)-pLBo(1))+pLBo(1)); %Mechanical trail
        
        %%% Steering Effort
        normalFx=cross( [1 0 0],pUBo-pLBo) ./ (norm( cross( [1 0 0],pUBo-pLBo) ) );
        normalFy=cross( [0 1 0],pUBo-pLBo) ./ (norm( cross( [0 1 0],pUBo-pLBo) ) );
        
        Modulus.Fx=cross(normalFx.*([Track/2+Scrub,Base+mechTrail,0]-pLBo),[1,0,0])...
                    ./cross(cross(pUBo-pLBo,pTBo-pTAo),(pTAo-pTBo)./norm(pTAo-pTBo));
                
        Modulus.Fy=cross(normalFy.*([Track/2+Scrub,Base+mechTrail,0]-pLBo),[0,1,0])...
                    ./cross(cross(pUBo-pLBo,pTBo-pTAo),(pTAo-pTBo)./norm(pTAo-pTBo));
                
        Modulus.Mz=cosd(KPI)./cross(cross(pUBo-pLBo,pTBo-pTAo),(pTAo-pTBo)./norm(pTAo-pTBo));
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

    function [Scrub, Trail] = KPIIntersection(pUB, pLB, Track, Base)
        pLBt = oTb( bTla( [0 Design.L.LA 0]', beta(1) ), Attitude.Roll, Attitude.Pitch );
        pUBt = oTb( bTua( [0 Design.L.UA 0]', beta(2) ), Attitude.Roll, Attitude.Pitch );
    end
end