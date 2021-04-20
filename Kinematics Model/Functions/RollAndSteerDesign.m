function [Target, Points, Geometry, Design] = RollAndSteerDesign( ...
    Target, Bounds, N )

%% Calculation Static FVSA and Instant Center
Target.FVSA = 1 ./ ( tan( abs( Target.CamberGain ) ) ); % Front View Swing Arm (FVSA) [mm]

WheelCircle = @(z) -sqrt( Target.FVSA.^2 - ( z - Target.Rl ).^2 ) + Target.Track ./ 2; % Circle Traced by FVSA
ForceLine = @(z) ( z - Target.RollCenter ) .* ( -Target.Track ./ (2.*Target.RollCenter) ); % Desired Force Line (CP -> IC)

Target.InstantCenter(2) = fsolve( @(z) WheelCircle(z) - ForceLine(z), 0, optimset('Display','off') ); % Solve Intersection Problem
Target.InstantCenter(1) = WheelCircle( Target.InstantCenter(2) ); % Static Instant Centers [mm,mm]

%% Generate Potential A-Arm & Tie Rod Points  
% Get Relevant Lower & Upper Bounds
LowerBound = [ Bounds.TA(:,1); Bounds.LB(2:3,1); Bounds.UB(2:3,1); Bounds.TB(1:2,1) ];
UpperBound = [ Bounds.TA(:,2); Bounds.LB(2:3,2); Bounds.UB(2:3,2); Bounds.TB(1:2,2) ];

% Generate Design Latin Hypercube
LHSNorm = lhsdesign( N.Design, length(LowerBound) );
for i = 1 : N.Design
    Points.LA.B(:,i) = zeros(1,3);
    Points.UA.B(:,i) = zeros(1,3);
    Points.TA.B(:,i) = LowerBound(1:3) + LHSNorm(i,1:3)' .* (UpperBound(1:3) - LowerBound(1:3));

    Points.LB.T(:,i) = [0; LowerBound(4:5) + LHSNorm(i,4:5)' .* (UpperBound(4:5) - LowerBound(4:5)) ];
    Points.UB.T(:,i) = [0; LowerBound(6:7) + LHSNorm(i,6:7)' .* (UpperBound(6:7) - LowerBound(6:7)) ];
    Points.TB.T(:,i) = [LowerBound(8:9) + LHSNorm(i,8:9)' .* (UpperBound(8:9) - LowerBound(8:9)); 0 ];
end

%% Generate World Coordinate Transformations
% Transform from World Coordinate to Specific Frame
wTb = @(p, L, zr, theta, phi) RotX(phi)*RotY(theta) * ( p + [L/2 0 0]') + [0 0 zr]';
wTt = @(p, delta, gamma, phi, L, Tw, Re) RotZ(delta)*RotX(gamma)*RotY(-phi) * p + [L/2, Tw/2, Re]';

% Transform to World Coordinate from Specfic Frame
bTw = @(p, L, zr, theta, phi) (RotX(phi)*RotY(theta) \ ( p - [0 0 zr]' )) - [L/2 0 0]'; 
tTw = @(p, delta, gamma, phi, L, Tw, Re) ( RotZ(delta)*RotX(gamma)*RotY(-phi) ) \ ( p - [L/2 Tw/2 Re]' );

%% Apply Design Constraints
if strcmp( Target.Axle, 'Front')
    L = Target.Wheelbase;
else
    L = -Target.Wheelbase;
end

Points.LA.W = wTb( Points.LA.B, L, Target.Ride, Target.Rake, 0 );
Points.UA.W = wTb( Points.UA.B, L, Target.Ride, Target.Rake, 0 );
Points.TA.W = wTb( Points.TA.B, L, Target.Ride, Target.Rake, 0 );
Points.LB.W = wTt( Points.LB.T, Target.Toe, Target.Camber, Target.Caster, ...
    L, Target.Track, Target.Rl );
Points.UB.W = wTt( Points.UB.T, Target.Toe, Target.Camber, Target.Caster, ...
    L, Target.Track, Target.Rl );
Points.TB.W = wTt( Points.TB.T, Target.Toe, Target.Camber, Target.Caster, ...
    L, Target.Track, Target.Rl );

% Find Inboard A-Arm Pickups (via Target Instant Center & Monocoque Surface)
InboardAArmY = @(y, pB, IC, TA, d) (tand(90-d)*(y - TA(2,:)) + TA(3,:)) - ...
    ((pB(3,:)-IC(2)) ./ (pB(2,:)-IC(1)) .* (y - pB(2,:)) + pB(3,:));
InboardAArmZ = @(y, pB, IC) (pB(3,:)-IC(2)) ./ (pB(2,:)-IC(1)) .* (y - pB(2,:)) + pB(3,:);

Points.LA.W(2,:) = fsolve( @(y) InboardAArmY( y, Points.LB.W, ...
    Target.InstantCenter, Points.TA.W, Target.DraftAngle), Points.TA.W(2,:), optimset('Display','off') );
Points.UA.W(2,:) = fsolve( @(y) InboardAArmY( y, Points.UB.W, ...
    Target.InstantCenter, Points.TA.W, Target.DraftAngle), Points.TA.W(2,:), optimset('Display','off') );

Points.LA.W(3,:) = InboardAArmZ( Points.LA.W(2,:), Points.LB.W, Target.InstantCenter );
Points.UA.W(3,:) = InboardAArmZ( Points.UA.W(2,:), Points.UB.W, Target.InstantCenter );

% Align A-Arms with Outboard Ball Joints
Points.LA.W(1,:) = (Points.LA.W(3,:) - Points.LB.W(3,:)) .* tand( Target.Rake ) + Points.LB.W(1,:);
Points.UA.W(1,:) = (Points.UA.W(3,:) - Points.UB.W(3,:)) .* tand( Target.Rake ) + Points.UB.W(1,:);

% Find Vertical Dimension of Outboard Tie Rod Pickups (via Target Instant Center)
OutboardTieRodZ = @(pA, pB, IC) (pA(3,:) - IC(2))./(pA(2,:) - IC(1)) .* (pB(2,:) - pA(2,:)) + pA(3,:);

Points.TB.W(3,:) = OutboardTieRodZ( Points.TA.W, Points.TB.W, Target.InstantCenter );

% Ensure Rear Tie Rod is Parallel to Tranverse Plane
if strcmp(Target.Axle, 'Rear')
    Points.TA.W(1,:) = Points.TB.W(1,:);
end

Points.LA.B = bTw( Points.LA.W, L, Target.Ride, Target.Rake, 0 );
Points.UA.B = bTw( Points.UA.W, L, Target.Ride, Target.Rake, 0 );
Points.TA.B = bTw( Points.TA.W, L, Target.Ride, Target.Rake, 0 );

Points.TB.T = tTw( Points.TB.W, Target.Toe, Target.Camber, Target.Caster, ...
    L, Target.Track, Target.Rl );

% Calculate Member Lengths
Geometry.Length.LA = sqrt( sum((Points.LA.W - Points.LB.W).^2) );
Geometry.Length.UA = sqrt( sum((Points.UA.W - Points.UB.W).^2) );
Geometry.Length.TR = sqrt( sum((Points.TA.W - Points.TB.W).^2) );

% Calculate KPI & Scrub
KPICalculation   = @(pL, pU) -atand( (pU(2,:) - pL(2,:))./(pU(3,:) - pL(3,:)) );
ScrubCalculation = @(pL, pU, Tw, Re) Tw/2 - ((pU(2,:) - pL(2,:))./(pU(3,:) - pL(3,:)) .* ( -pL(3,:) ) + pL(2,:));

Geometry.KPI = KPICalculation( Points.LB.W, Points.UB.W );

Geometry.Scrub = ScrubCalculation( Points.LB.W, Points.UB.W, Target.Track, Target.Rl );

% Store Relevant Points & Geometry in Design
for i = 1 : size(Points.LA.B, 2)
    Design(i).p.LAb = Points.LA.B(:,i);
    Design(i).p.UAb = Points.UA.B(:,i);
    Design(i).p.TAb = Points.TA.B(:,i);
    
    Design(i).p.LBt = Points.LB.T(:,i);
    Design(i).p.UBt = Points.UB.T(:,i);
    Design(i).p.TBt = Points.TB.T(:,i);
    
    Design(i).L.LA = Geometry.Length.LA(i);
    Design(i).L.UA = Geometry.Length.UA(i);
    Design(i).L.TR = Geometry.Length.TR(i);
    
    Design(i).a.LA = [0, 0];
    Design(i).a.UA = [0, 0];
end

%% Local Functions
    function R = RotX( Alpha )
        R = [1 0 0; 0 cosd(Alpha) sind(Alpha); 0 -sind(Alpha) cosd(Alpha)];
    end

    function R = RotY( Beta )
        R = [cosd(Beta) 0 sind(Beta); 0 1 0; -sind(Beta) 0 cosd(Beta)];
    end

    function R = RotZ( Gamma )
        R = [cosd(Gamma) sind(Gamma) 0; -sind(Gamma) cosd(Gamma) 0; 0 0 1];
    end
end