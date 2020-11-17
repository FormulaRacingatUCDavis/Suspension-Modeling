clc; clear; close all;

%% Suspension Kinematics Design Script
% This script solves systems of kinematics equations to allow for an
% improved and simplified kinematics workflow for mid fidelity suspension
% design. This will allow for quick and intuitive design space exploration.
% 
% Requires:
% - Optimization Toolbox
% 
% Blake Christierson - bechristierson@ucdavis.edu
%
% Version
% 1.0 - Basic Model w/o Spring Geometry
% 2.0 - Reformatted Script Structure
% 3.0 - Vector Loop Solving Procedure

%% Boilerplate
% Option to Load Previous Designs
cd( fileparts( which( 'SuspensionKinematicModel.m' ) ) )

File.Name = uigetfile('*.mat','Select Previously Generated Data or Skip to Generate New Data' );

if File.Name~= 0
    load(File.Name);
else    
    %% User Inputs
    DesignAxle = [1]; % 1 - Front Axle
                      % 2 - Rear Axle
                      % [1 2] - Both Axles
    
    %%% Vehicle Level Targets
    Target.Wheelbase  = 1525;             % Nominal Wheelbase [mm]
    Target.WeightDist = 0.5;              % Static Front Weight Distribution []
    Target.CG(3)      = 8.5  .* (25.4);   % Nominal CG Height [in -> mm]
    Target.Ride       = 2    .* (25.4);   % Nominal Ride Height [in -> mm]
    Target.Rake       = 0;                % Nominal Rake Angle [deg]
    Target.Rl         = 7.85 .* (25.4);   % Nominal Loading Radius [in -> mm]
    
    Target.CG(1) = Target.Wheelbase * (1-Target.WeightDist); % C.G. to Front Axle (a) [mm]
    
    Target.Axle = 'Front';
    Target(2) = Target(1); 
    Target(2).Axle = 'Rear';
    Target(2).WeightDist = 1 - Target(1).WeightDist; % Static Rear Weight Distribution []
    Target(2).CG(1) = Target.Wheelbase - Target(1).CG(1); % C.G. to Rear Axle (b) [mm]
    
    %%% Suspension Objectives
    Target(1).Track       =  1220;                    % Nominal Front Track Width [mm]
    Target(1).RollCenter  =  2.00 .* (25.4);          % Force-Based Roll Center Height [in -> mm]
    Target(1).DraftAngle  =  3.00;                    % Draft Angle [deg]
    Target(1).Caster      =  3.00;                    % Caster [deg]
    Target(1).Camber      = -1.20;                    % Static Camber [deg]
    Target(1).CamberGain  = -1.50 .* (pi/180 / 25.4); % Camber Gain [deg/in -> rad/mm]
    Target(1).Toe         =  0.50;                    % Static Toe (Positive Out) [deg]
    Target(1).Scrub       =  0.50 .* (25.4);          % Maximum Scrub [in -> mm]
    Target(1).KPI         =  5.00;                    % Maximum KPI [deg]
    Target(1).MotionRatio =  0.80;                    % Motion Ratio Target [] 
    
    Target(2).Track       =  1220;                    % Nominal Rear Track Width [mm] 
    Target(2).RollCenter  =  4.00 .* (25.4);          % Force-Based Roll Center Height [in -> mm]
    Target(2).DraftAngle  =  3.00;                    % Draft Angle [deg]
    Target(2).Caster      =  0.00;                    % Caster [deg]
    Target(2).Camber      = -0.70;                    % Static Camber [deg]
    Target(2).CamberGain  = -1.20 .* (pi/180 / 25.4); % Camber Gain [deg/in -> rad/mm]
    Target(2).Toe         = -0.50;                    % Static Toe (Positive Out) [deg]
    Target(2).Scrub       =  0.25 .* (25.4);          % Maximum Scrub [in -> mm]
    Target(2).KPI         = 15.00;                    % Maximum KPI [deg]
    Target(2).MotionRatio =  0.80;                    % Motion Ratio Target [] 
    
    %%% Suspension Hard Point Bounds
    % All values specified by single 0's are not required and should not be
    % changed. These coordinates are automatically solved for via design
    % rules.
    
    % Inboard Pickups: Longitudinal |   Lateral   |   Vertical  | 
    Bounds(1).LA =    [  0   ,  0   ;  0   ,  0   ;  0.50,  2.00] .* (25.4); % FLA Bounds (BCS) [in -> mm]
    Bounds(1).UA =    [  0   ,  0   ;  0   ,  0   ;  4.50, 10.00] .* (25.4); % FUA Bounds (BCS) [in -> mm]
    Bounds(1).TA =    [  0.00,  4.00;  8.70,  8.70;  2.50,  4.00] .* (25.4); % FTA Bounds (BCS) [in -> mm]
    Bounds(1).RA =    [- 3.00,  0.00;  6.00,  9.50;  0.50, 10.00] .* (25.4); % FRA Bounds (BCS) [in -> mm]
    Bounds(1).PA =    [  0   ,  0   ;  2.00,  4.00;  0   ,  0   ] .* (25.4); % FPA Bounds (RCS) [in -> mm]
    Bounds(1).SA =    [- 3.00,  0.00;  8.00, 12.00; 10.00, 18.00] .* (25.4); % FSA Bounds (BCS) [in -> mm]

    Bounds(2).LA =    [  0   ,  0   ;  0   ,  0   ;  0.50,  2.00] .* (25.4); % RLA Bounds (BCS) [in -> mm]
    Bounds(2).UA =    [  0   ,  0   ;  0   ,  0   ;  4.50, 10.00] .* (25.4); % RUA Bounds (BCS) [in -> mm]
    Bounds(2).TA =    [  2.25,  3.75;  8.70,  9.25;  4.00,  5.00] .* (25.4); % RTA Bounds (BCS) [in -> mm]
    Bounds(2).RA =    [- 2.00,  2.00;  6.00,  9.50;  0.50, 10.00] .* (25.4); % RRA Bounds (BCS) [in -> mm]
    Bounds(2).PA =    [  0   ,  0   ;  2.00,  4.00;  0   ,  0   ] .* (25.4); % RPA Bounds (RCS) [in -> mm]
    Bounds(2).SA =    [- 3.00,  3.00;  8.00, 12.00; 10.00, 18.00] .* (25.4); % RSA Bounds (BCS) [in -> mm]

   % Outboard Pickups: Longitudinal |   Lateral   |  Vertical   |
    Bounds(1).LB =    [  0   ,  0   ;- 0.85,- 0.85;- 3.30,- 2.85] .* (25.4); % FLB Bounds (TCS) [in -> mm] 
    Bounds(1).UB =    [  0   ,  0   ;- 2.00,- 0.85;  2.60,  3.70] .* (25.4); % FUB Bounds (TCS) [in -> mm] 
    Bounds(1).TB =    [  2.25,  3.75;- 1.75,- 0.85;- 2.50,  2.50] .* (25.4); % FTB Bounds (TCS) [in -> mm]
    Bounds(1).PB =    [  2.25,  3.75;- 1.75,- 0.85;- 2.50,  2.50] .* (25.4); % FPB Bounds (LACS) [in -> mm] 
    Bounds(1).SB =    [  2.25,  3.75;- 1.75,- 0.85;- 2.50,  2.50] .* (25.4); % FSB Bounds (RCS) [in -> mm]

    Bounds(2).LB =    [  0   ,  0   ;- 0.85,- 0.85;- 3.30,- 2.85] .* (25.4); % RLB Bounds (TCS) [in -> mm] 
    Bounds(2).UB =    [  0   ,  0   ;- 2.00,- 0.85;  2.60,  3.70] .* (25.4); % RUB Bounds (TCS) [in -> mm] 
    Bounds(2).TB =    [  2.25,  3.75;- 1.75,- 0.85;- 2.50,  2.50] .* (25.4); % RTB Bounds (TCS) [in -> mm]
    Bounds(2).PB =    [  2.25,  3.75;- 1.75,- 0.85;- 2.50,  2.50] .* (25.4); % RPB Bounds (LACS) [in -> mm] 
    Bounds(2).SB =    [  2.25,  3.75;- 1.75,- 0.85;- 2.50,  2.50] .* (25.4); % RSB Bounds (RCS) [in -> mm]
    
    %%% Operating Condition Ranges
    Attitude.Ride  = [  1.5  2.5] .* (25.4); % Ride Height Range [mm]
    Attitude.Pitch = [ -3.0  3.0];           % Pitch Range [deg]
    Attitude.Roll  = [  0.0  3.0];           % Roll  Range (Mirrored) [deg]
    Attitude.Steer = [-15.0 25.0];           % Steer Range (Mirrored) [deg]

    N.Design = 1; % Number of Designs
    
    %% Design Roll & Steer Geometry
    for i = DesignAxle
        Target(i).FVSA = [];
        Target(i).InstantCenter = [];
        
        [Target(i), Points(i), Geometry(i), Design(i,:)] = ...
            RollAndSteerDesign( Target(i), Bounds(i), N );
    end
end

%% Plotting Roll & Steer Geometry
Test = RollAndSteerAnalysis( Target, Points, Design );

return
    
    %% Buckling & Clearance Studies
    
%% Old Material


    %% Optimization of A-Arms & Tie Rods
    %%% Set Up Design Space & Operation Space
    for i = 1 : N.Design
        Design(i).FLA = Points.FLA.B(:,i); %#ok<*SAGROW>
        Design(i).FUA = Points.FUA.B(:,i);
        Design(i).FTA = Points.FTA.B(:,i);
        Design(i).FLB = Points.FLB.T(:,i);
        Design(i).FUB = Points.FUB.T(:,i);
        Design(i).FTB = Points.FTB.T(:,i);

        Design(i).RLA = Points.RLA.B(:,i);
        Design(i).RUA = Points.RUA.B(:,i);
        Design(i).RTA = Points.RTA.B(:,i);
        Design(i).RLB = Points.RLB.T(:,i);
        Design(i).RUB = Points.RUB.T(:,i);
        Design(i).RTB = Points.RTB.T(:,i);

        Design(i).Length.FLA = Geometry.Length.FLA(i);
        Design(i).Length.FUA = Geometry.Length.FUA(i);
        Design(i).Length.FTR = Geometry.Length.FTR(i);

        Design(i).Length.RLA = Geometry.Length.RLA(i);
        Design(i).Length.RUA = Geometry.Length.RUA(i);
        Design(i).Length.RTR = Geometry.Length.RTR(i);
    end

    LHSNorm = lhsdesign( 4, N.Operation );
    for j = 1 : size( LHSNorm, 2 )
        Attitude.Sample(j).Ride  = Attitude.Ride(1)  + LHSNorm(1,j)' .* diff( Attitude.Ride );  
        Attitude.Sample(j).Pitch = Attitude.Pitch(1) + LHSNorm(2,j)' .* diff( Attitude.Pitch );
        Attitude.Sample(j).Roll  = Attitude.Roll(1)  + LHSNorm(3,j)' .* diff( Attitude.Roll );
        Attitude.Sample(j).Steer = Attitude.Steer(1) + LHSNorm(4,j)' .* diff( Attitude.Steer ); 

    end

    clear i j LHSNorm

    for i = 1 : numel( Design )
        tic;
        for j = 1 : numel( Attitude.Sample )
            [ Response(i).Rake(:,j)         , Response(i).Track(:,j)     , ...
              Response(i).Caster(:,j)       , Response(i).Camber(:,j)    , Response(i).Toe(:,j), ...
              Response(i).InstantCenter(:,:,j), Response(i).RollCenter(:,j), ...
              Response(i).KPI(:,j)          , Response(i).Scrub(:,j)     ] = ...
                    TireOrientation( Design(i), Attitude.Sample(j), Target, Transform );
        end
        toc
    end

    for i = 1 : N.Operation
        Attitude.Sample(N.Operation+j).Ride  = Attitude.Sample(j).Ride;  
        Attitude.Sample(N.Operation+j).Pitch = Attitude.Sample(j).Pitch;
        Attitude.Sample(N.Operation+j).Roll  = -Attitude.Sample(j).Roll;
        Attitude.Sample(N.Operation+j).Steer = -Attitude.Sample(j).Steer; 
    end

    LinearModel = fitlm( [vertcat( Attitude.Sample.Ride ), vertcat( Attitude.Sample.Pitch ), ...
         vertcat( Attitude.Sample.Roll )], Response(i).Caster );

    figure; 
    for i = 1:N.Design
        %scatter3( [Attitude.Sample.Pitch], [Attitude.Sample.Roll], Response(i).RollCenter ); hold on;
        scatter3( [Attitude.Sample.Pitch], [Attitude.Sample.Roll], Response(i).Camber ); hold on;
        %scatter3( [Attitude.Sample.Pitch], [Attitude.Sample.Roll], Response(i).Scrub ); hold on;
    end
    xlabel( 'Pitch' ); ylabel( 'Roll' ); zlabel( 'Roll Center' );

%% Plotting & Data Analysis
Label = fieldnames( Attitude.Sample );
Marker = 'o+*.x_|sd^v><ph';

%%% Instant Center Plot
for i = 1:numel(Label)
    subplot(2,2,i)
    for j = 1:numel(Marker)
        if i <= 2
            scatter( reshape(Response(j).InstantCenter(1:2,2,:),[],1), ...
                     reshape(Response(j).InstantCenter(1:2,3,:),[],1), ...
                     25, repmat(vertcat(Attitude.Sample.(Label{i})),2,1), Marker(j) )
        else
            scatter( reshape(Response(j).InstantCenter(1:2,2,:),[],1), ...
                     reshape(Response(j).InstantCenter(1:2,3,:),[],1), ...
                     25, [vertcat(Attitude.Sample.(Label{i})); -vertcat(Attitude.Sample.(Label{i}))], Marker(j) )
        end
        hold on
    end
    
    xlabel( 'Instant Center y-Coordinate [mm]' )
    ylabel( 'Instant Center z-Coordinate [mm]' )
    axis equal
    
    cb = colorbar;
    cb.Label.String = Label{i};
end

return

figure;
for i = 1 : p^2
    Axes = subplot(p,p,i);
    
    [jj, ii] = ind2sub( [p p], i );
    
    % Plotting Data
    if ii == jj
        histogram( Data(:,ii,1), 10 )
        hold on
        histogram( Data(:,ii,end), 10 )
    else
        for j = 1 : n
           scatter( squeeze(Data(j,jj,:)), squeeze(Data(j,ii,:)), '.' );
           hold on
        end
    end
    
    % Manipulating Axes Object
    if jj == 1
        Axes.YLabel.String = 'name';
    else
        Axes.YTick = [];
        Axes.YTickLabel = [];
    end
    
    if ii == p
        Axes.XLabel.String = 'name';
    else
        Axes.XTick = [];
        Axes.XTickLabel = [];
    end
end

%% Local Functions


%{
function [x, F, ExitFlag, Output] = RollOptimization( x0, Model, Target, Bounds, Options )
    LB = [ Bounds.LA(1,:,1)' ; Bounds.UA(1,:,1)' ; Bounds.LB(1,:,1)' ; Bounds.UB(1,:,1)' ; ...
       Bounds.LA(2,:,1)' ; Bounds.UA(2,:,1)' ; Bounds.LB(2,:,1)' ; Bounds.UB(2,:,1)' ];
       
    UB = [ Bounds.LA(1,:,2)' ; Bounds.UA(1,:,2)' ; Bounds.LB(1,:,2)' ; Bounds.UB(1,:,2)' ; ...
       Bounds.LA(2,:,2)' ; Bounds.UA(2,:,2)' ; Bounds.LB(2,:,2)' ; Bounds.UB(2,:,2)' ];
   
    xi = [];
    Fi = [];
    Ci = [];
    Ceqi = [];
    
    ObjFun = @(x) ObjectiveFunction( x, Model, Target );
    ConFun = @(x) ConstraintFunction( x, Model, Target );
    
    [x, F, ExitFlag, Output] = patternsearch( ObjFun, x0, [], [], [], [], LB, UB, ConFun, Options );
    
    function Fval = ObjectiveFunction( x, Model, Target )
        if ~isequal( x, xi )
           [Fi, Ci, Ceqi] = RollPointsFunctions( x, Model, Target );
           xi = x;
        end
        
        Fval = Fi;
    end
    
    function [C, Ceq] = ConstraintFunction( x, Model, Target )
        if ~isequal( x, xi )
           [Fi, Ci, Ceqi] = RollPointsFunctions( x, Model, Target );
           xi = x;
        end
        
        C = Ci;
        Ceq = Ceqi; 
    end

    function [Fval, C, Ceq] = RollPointsFunctions( x, Model, Target )
        load_system( Model );

        %%% Assign Design Variables to Base Workspace
        assignin( 'base', 'FLA', x( 1:3 ) );
        assignin( 'base', 'FUA', x( 4:6 ) );
        assignin( 'base', 'FLB', x( 7:9 ) );
        assignin( 'base', 'FUB', x(10:12) );
        assignin( 'base', 'RLA', x(13:15) );
        assignin( 'base', 'RUA', x(16:18) );
        assignin( 'base', 'RLB', x(19:21) );
        assignin( 'base', 'RUB', x(22:24) );

        try
            %%% Run Simulation
            set_param( Model, 'FastRestart', 'off' );
            set_param( Model, 'SimMechanicsOpenEditorOnUpdate', 'off' );

            Results = sim( Model );

            %%% Parse Results
            RollArm = [ Results.Fd.signals.values(end,:) ; ...
                        Results.Rd.signals.values(end,:) ] .* 1000;
            FVSA = [ Results.FFVSA.signals.values(end,:) ; ...
                     Results.RFVSA.signals.values(end,:) ] .* 1000;
            KPI = [ Results.FKPI.signals.values(end,:) ; ...
                    Results.RKPI.signals.values(end,:) ];
            Scrub = [ Results.Fs.signals.values(end,:) ; ...
                  Results.Rs.signals.values(end,:) ] .* 1000;

            %%% Compute Cost Function
            Weight = [ 0.35 ; ... % Roll Arm Target
                       0.35 ; ... % FVSA Length Target
                       0.30 ];    % [KPI or Scrub] Target

            Fval = Weight(1) .* abs( (RollArm - Target.RollArm) ./ Target.RollArm ) + ...
                Weight(2) .* abs( (FVSA    - Target.FVSA)    ./ Target.FVSA )    + ...
                Weight(3) .* abs( [ ( deg2rad(KPI(1)) - Target.Caster(1) ) ./ Target.Caster(1); ...
                                    Scrub(2) ./ Target.Scrub(2) ] );

            Fval = sum( Fval );

            %%% Compute Constraints
            % Front Scrub Constraints
            C(1) = -Scrub(1); % Positive Scrub
            C(2) = Scrub(1) - Target.Scrub(1); % Less Than Maximum Scrub

            % Rear KPI Constraints
            C(3) = -KPI(2); % Positive KPI
            C(4) = KPI(2) - Target.KPI(2); % Less Than Maximum KPI
        catch
            Fval = NaN;
            C(1:4) = 0.01;
        end
        
        Ceq = [];
    end
end
%}