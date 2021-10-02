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
% Stuart Scolaro - slscolaro@ucdavis.edu

addpath( genpath( [fileparts( which( 'SuspensionKinematicModel.m' ) ), '\Functions'] ) )
addpath( genpath( fileparts( which( 'ContactPatchLoads.m' ) ) ) )

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
    Target.SprungMass = 225;              % Sprung Mass [kg]
    Target.CG(3)      = 10.15  .* (25.4);   % Nominal CG Height [in -> mm]
    Target.Ride       = 2    .* (25.4);   % Nominal Ride Height [in -> mm]
    Target.Rake       = 0;                % Nominal Rake Angle [deg]
    Target.Rl         = 7.85 .* (25.4);   % Nominal Loaded Radius [in -> mm]
    %Target.TireParameter = load('Hoosier_R25B_16x75-10x7.mat'); %Tire Data
    
    Target.CG(1) = Target.Wheelbase * (1-Target.WeightDist); % C.G. to Front Axle (a) [mm]
    
    Target.Axle = 'Front';
    Target(2) = Target(1); 
    Target(2).Axle = 'Rear';
    Target(2).WeightDist = 1 - Target(1).WeightDist; % Static Rear Weight Distribution []
    Target(2).CG(1) = Target(1).Wheelbase - Target(1).CG(1); % C.G. to Rear Axle (b) [mm]
    
    %%% Suspension Objectives
    Target(1).Track       =  1220;                    % Nominal Front Track Width [mm]
    Target(1).RollCenter  =  1.00 .* (25.4);          % Force-Based Roll Center Height [in -> mm]
    Target(1).DraftAngle  =  5.00;                    % Draft Angle [deg]
    Target(1).Caster      =  3.00;                    % Caster [deg]
    Target(1).Camber      = -1.60;                    % Static Camber [deg]
    Target(1).CamberGain  = -0.75 .* (pi/180 / 25.4); % Camber Gain [deg/in -> rad/mm]
    Target(1).Toe         =  0.50;                    % Static Toe (Positive Out) [deg]
    Target(1).Scrub       =  0.50 .* (25.4);          % Maximum Scrub [in -> mm]
    Target(1).KPI         =  8.00;                    % Target KPI [deg]
    Target(1).MotionRatio =  0.80;                    % Motion Ratio Target [] 
    
    Target(2).Track       =  1220;                    % Nominal Rear Track Width [mm] 
    Target(2).RollCenter  =  3.00 .* (25.4);          % Force-Based Roll Center Height [in -> mm]
    Target(2).DraftAngle  = 15.00;                    % Draft Angle [deg]
    Target(2).Caster      =  0.00;                    % Caster [deg]
    Target(2).Camber      = -0.70;                    % Static Camber [deg]
    Target(2).CamberGain  = -0.25 .* (pi/180 / 25.4); % Camber Gain [deg/in -> rad/mm]
    Target(2).Toe         = -0.50;                    % Static Toe (Positive Out) [deg]
    Target(2).Scrub       =  0.25 .* (25.4);          % Target Scrub [in -> mm]
    Target(2).KPI         = 10.00;                    % Maximum KPI [deg]
    Target(2).MotionRatio =  0.80;                    % Motion Ratio Target [] 
    
    %%% Suspension Hard Point Bounds
    % All values specified by single 0's are not required and should not be
    % changed. These coordinates are automatically solved for via design
    % rules.
    
    % Inboard Pickups: Longitudinal |   Lateral   |   Vertical  | 
    Bounds(1).LA =    [  0   ,  0   ;  0   ,  0   ;  0   ,  0   ] .* (25.4); % FLA Bounds (XCS) [in -> mm]
    Bounds(1).UA =    [  0   ,  0   ;  0   ,  0   ;  0   ,  0   ] .* (25.4); % FUA Bounds (XCS) [in -> mm]
    Bounds(1).TA =    [  3.00,  3.00;  8.70,  8.70;  4.50,  4.50] .* (25.4); % FTA Bounds (XCS) [in -> mm]
    Bounds(1).RA =    [- 3.00,  2.00;  6.00,  9.50;  8.00, 10.00] .* (25.4); % FRA Bounds (XCS) [in -> mm]
    Bounds(1).PA =    [  0   ,  0   ;  2.00,  4.00;  0   ,  0   ] .* (25.4); % FPA Bounds (RCS) [in -> mm]
    Bounds(1).SA =    [- 3.00,  0.00;  8.00, 12.00; 10.00, 18.00] .* (25.4); % FSA Bounds (XCS) [in -> mm]

    Bounds(2).LA =    [  0   ,  0   ;  0   ,  0   ;  1.75,  1.75] .* (25.4); % RLA Bounds (XCS) [in -> mm]
    Bounds(2).UA =    [  0   ,  0   ;  0   ,  0   ; 10.50, 10.50] .* (25.4); % RUA Bounds (XCS) [in -> mm]
    Bounds(2).TA =    [  6.75,  6.75; 11.00, 11.00;  3.50,  3.50] .* (25.4); % RTA Bounds (XCS) [in -> mm]
    Bounds(2).RA =    [  4.00,  8.00;  8.00, 10.00;  9.00, 11.00] .* (25.4); % RRA Bounds (XCS) [in -> mm]
    Bounds(2).PA =    [  0   ,  0   ;  2.00,  4.00;  0   ,  0   ] .* (25.4); % RPA Bounds (RCS) [in -> mm]
    Bounds(2).SA =    [ 11.00, 11.00;  8.00, 12.00;  2.00,  4.00] .* (25.4); % RSA Bounds (XCS) [in -> mm]

   % Outboard Pickups: Longitudinal |   Lateral   |  Vertical   |
    Bounds(1).LB = RotY( Target(1).Caster ) * [  0   ,  0   ;- 0.87,- 0.87;- 2.75,- 2.75] .* (25.4); % FLB Bounds (WCS) [in -> mm]
    Bounds(1).UB = RotY( Target(1).Caster ) * [  0   ,  0   ;- 1.81,- 1.75; 3.375, 3.375] .* (25.4); % FUB Bounds (WCS) [in -> mm] 
    Bounds(1).TB = RotY( Target(1).Caster ) * [  2.98,  2.98;- 1.37,- 1.37;- 1.00,- 1.00] .* (25.4); % FTB Bounds (WCS) [in -> mm]
    Bounds(1).PB =    [  2.25,  3.75;- 1.75,- 0.85;- 2.50,  2.50] .* (25.4); % FPB Bounds (ACS) [in -> mm] 
    Bounds(1).SB =    [  2.25,  3.75;- 1.75,- 0.85;- 2.50,  2.50] .* (25.4); % FSB Bounds (RCS) [in -> mm]

    Bounds(2).LB =    [  0   ,  0   ;- 0.87,- 0.87;- 3.20,- 2.70] .* (25.4); % RLB Bounds (WCS) [in -> mm] 
    Bounds(2).UB =    [  0   ,  0   ;- 1.81,- 1.45;  2.60,  3.45] .* (25.4); % RUB Bounds (WCS) [in -> mm] 
    Bounds(2).TB =    [  2.25,  3.50;- 1.81,- 0.87;- 2.50,  2.50] .* (25.4); % RTB Bounds (WCS) [in -> mm]
    Bounds(2).PB =    [  2.25,  3.75;- 1.75,- 0.85;- 2.50,  2.50] .* (25.4); % RPB Bounds (ACS) [in -> mm] 
    Bounds(2).SB =    [  2.25,  3.75;- 1.75,- 0.85;- 2.50,  2.50] .* (25.4); % RSB Bounds (RCS) [in -> mm]
    
    %%% Operating Condition Ranges
    Attitude.Ride  = [  1.5  2.5] .* (25.4); % Ride Height Range [mm]
    Attitude.Pitch = [ -3.0  3.0];           % Pitch Range [deg]
    Attitude.Roll  = [  0.0  3.0];           % Roll  Range (Mirrored) [deg]
    Attitude.Steer = [-15.0 25.0];           % Steer Range (Mirrored) [in]

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
    
%% Buckling & Clearance Studies
    
%% Local Functions
    function R = RotY( Beta )
        R = [cosd(Beta) 0 sind(Beta); 0 1 0; -sind(Beta) 0 cosd(Beta)];
    end