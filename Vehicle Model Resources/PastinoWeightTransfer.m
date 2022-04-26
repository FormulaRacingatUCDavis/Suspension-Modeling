clc; clear; close all;
%Faustino's S-Tier Simple Weight Transfer Model 
%Contact: fblozis@ucdavis.edu
%%
%Define Inputs

m = 270 + .15*270;       %vehicle+driver mass (kg) + percent safety factor                                            
g = 9.81;     

a_y = 0.7 .* 9.81;        %lat accel in G's (CHANGED)                                                           
a_x = 0;          %long accel in G's                                                           

tw_f = 1.220;      %track width front (m)   
tw_r = tw_f;        %track width rear (m)
w_b = 1.550;      %wheel base (m)                                                           
h = .25;           %CG Height (m)                                                              

wd_f = .45:0.01:.55;    %weight distribution front %                                                             
wd_l = .45:0.01:.55;    %weight distribution left %                                                           
a = wd_f*w_b;
b = w_b - a;

mf = wd_f*m;     %mass on front axle
mr = m - mf;     %mass on rear axle

  

%% Longitudinal Weight Transfer 

dW_long = wd_f*m - (a*m - m*a_x*h)/w_b;

%% Lateral Weight Transfer for Front & Rear Axles 

dW_latf = (mf*a_y*h/tw_f);       %kg
dW_latr = (mr*a_y*h/tw_r);       %kg

%% Sum Forces

% 1: Front Left Corner
% 2: Front Right Corner
% 3: Rear Left Corner
% 4: Rear Right Corner

Load = zeros(4, length(wd_f));

for i = 1:length(wd_f)

Load(1,i) = m/4 + dW_latf(i) + -dW_long(i)/2;
Load(2,i) = m/4 - dW_latf(i) + -dW_long(i)/2;
Load(3,i) = m/4 + dW_latr(i) - -dW_long(i)/2; 
Load(4,i) = m/4 - dW_latr(i) - -dW_long(i)/2; 

end

x = Load*g;

%Top row: LLTD
%Next Four Rows: Load on each tire, in Newtons, 1,2,3,4
%zeros
%Next Four Rows: Load on each tire, in Kg, 1,2,3,4

variedFLLTDLoads = [wd_f; x; zeros(1, length(wd_f)); x/g];





%% Tire Model stuff



% SlipAngle = linspace(-12,12,50);
% SlipRatio = linspace(-0.15,0.15,50);
% [SlipAngle, SlipRatio] = meshgrid(SlipAngle, SlipRatio);
% SlipAngle = SlipAngle(:);
% SlipRatio = SlipRatio(:);
% Fidelity.Pure = 'Pacejka'; Fidelity.Combined = 'MNC';
% [Fx, Fy, ~, ~, ~] = Model.ContactPatchLoads( SlipAngle, SlipRatio, 1200, 10, 0, 10, 1, Fidelity); 
