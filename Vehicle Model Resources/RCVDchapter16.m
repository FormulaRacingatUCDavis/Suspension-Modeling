%parameters

w1 = 1020; %lbs
w2 = 1020; 
w3 = 680;
w4 = 680;
wF = 2040;
wR_t = 1360;
wT = 3400;
tF = 5; %ft
tR = 5;
L = 9.5;
h = 1.5; %CG height
H = 1.1; %CG to Roll axis 
alpha = -10; %degrees
R = -600; %ft
V = 146.7; %ft/sec
zRF = 0.25; %ft
zRR = 0.625;
%Zero drive torque and longitudinal acc
TD = 0.0; 
Ax = 0.0; 
%Roll rtes 
KF = 1222; %lb.-ft/deg
KR = 873; 
rideTravel = 2.5; %inches - minimum ride travel

%Equation for CG position 
b = wF*L/wT;
a = L - b;

%Lateral acceleartion values rleative to the Earth 
%Aalpha = horizontal lateral acceleartion
Aalpha = V^2 / (R*32.2); 
Agamma = (Aalpha * cosd(alpha)) - sind(alpha); 

%Effective weight of the car due to the banking 
Wprime = wT*((Aalpha*sind(alpha)+cosd(alpha)));

%Effective front and rear ale weights 
wFprime = (Wprime * b) / L;
wRprime = (Wprime*a) / L; 

%Roll gradient - phi is body roll angle
rollgradient = (-wT*H) / (KF + KR); %deg./g

%Weight transfer 
wF_t = Agamma * wT/tF * ((H*KF)/(KF + KR) + b/L * zRF);
wR_t = Agamma * wT/tR * ((H*KR)/(KF + KR) + a/L * zRR); %lbs

%Tire loads (corner) 
wFO = wFprime/2 + -wF_t; 
wFI = wFprime/2 - -wF_t; 
wRO = wRprime/2 + -wR_t; 
wRI = wRprime/2 - -wR_t; 

%Change from static loads measured on level ground
delta_wFO = wFO - w1; 
delta_wFI = wFI - w2; 
delta_wRO = wRO - w3; 
delta_wRI = wRI - w4; 

%Ride frequencies
KRF = delta_wFO / rideTravel;
KRR = delta_wRO / rideTravel; 

omegaF = 1/(2*pi) * sqrt((KRF * 12 * 32.2)/w2); %Hz
omegaFcpm = omegaF * 60; 
omegaR = 1/(2*pi) * sqrt((KRR * 12 * 32.2)/w4); 
omegaRcpm = omegaR * 60; F



