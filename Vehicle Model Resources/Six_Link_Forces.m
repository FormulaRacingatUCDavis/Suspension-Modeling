W = 530; %Weight (lbf)
WDF = 0.4; %Weight Distribution Front (%)
WDR = 0.6; %Weight Distribution Rrear (%)
L = 61; %Wheelbase(in.)
h = 12; %CG Height – h (in.) 12
r = 10; %Wheel Radius – r (in.) 10
Wfs = 212; %Static Weight Front – Wfs (lbf) 212
Wrs = 318; %Static Weight Rear – Wrs (lbf) 318
c = 24.4; %CG to Rear Axle – c (in.) 24.4
b = 26.6; %Front Axle to CG – b (in.) 36.6

% Additional Excel File for Calculations

A = ones(6,6);
B = ones(6,1);

x = A\b;