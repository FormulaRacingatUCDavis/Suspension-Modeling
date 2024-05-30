clc; clear; close all;

weight =[0,5.2,10.2,20.2,29.4,38.2];
run_1 = [2962,2964,2969,2980,2990,3001];
run_2 = [2955,2961,2966,2978,2989,3001];
P_1 = polyfit(weight,run_1,1);
P_2 = polyfit(weight,run_2,1);
yfit_1 = P_1(1)*weight+P_1(2);
yfit_2 = P_2(1)*weight+P_2(2);

figure(1)
hold on
title('Mass (lbs) vs Signal')
ylabel('Signal')
xlabel('Mass (lbs)')
plot(weight,run_1)
plot(weight,run_2)
plot(weight,yfit_1,'b-.')
plot(weight,yfit_2,'r-.')
legend('Run 1','Run 2','Linear Fit 1','Linear Fit 2')

T = readtable('Conner_Skidpad_Strain_Gauge.csv','NumHeaderLines',1);
figure(2)
Time = table2array(T(:,1));
Real_Readings = table2array(T(:,2));
Smoothed_Data_Real = smoothdata(Real_Readings,"gaussian");
Conversion = (Real_Readings-P_1(2))./P_1(1);
Smoothed_Conversion = smoothdata(Conversion,"gaussian");
%p_3 = polyfit(Time,Conversion,30);
%f1 = polyval(p_3,Time);
g_force = ((Smoothed_Conversion+120).*32.174)./(Smoothed_Conversion.*32.174);
subplot(2,2,1)
hold on
plot(Time,Smoothed_Data_Real)
%plot(Time,Real_Readings)
title('Time vs Signal')
ylabel('Signal')
xlabel('Time')
subplot(2,2,2)
hold on
plot(Time,Smoothed_Conversion)
%plot(Time,Conversion)
title('Time vs Mass(lbs)')
ylabel('Mass (lbs)')
xlabel('Time')
subplot(2,2,3:4)
plot(Time,g_force)
title('Time vs Vertical Force(Gs)')
ylabel('Force (Gs)')
xlabel('Time')

figure(3);
hold on
plot(Time,Smoothed_Data_Real,'r-');
plot(Time,Real_Readings,'b-');
legend('one','two');
title('Comparison of Smoothing Methods');
ylabel('Signal');
xlabel('Time');

%% Thunderhill data
data = readmatrix('001_brandon_parsed.csv');

% Find the rows where column 2 values are between the thresholds
rows = data(:, 1) == 500;

% Extract the corresponding values from column 10
filteredData = data(rows, [2,10]);
StrainGaugeData = filteredData(:,1);
TimeData = filteredData(:,2);

figure(4)
hold on
%plot(filteredData(:,2),filteredData(:,1))
Smoothed_Data_Real = smoothdata(StrainGaugeData);
i = 0;
while i <= 1
    Smoothed_Data_Real = smoothdata(Smoothed_Data_Real);
    i = i+1;
end
Conversion = (StrainGaugeData-P_1(2))./P_1(1);
Smoothed_Conversion = smoothdata(Conversion);
i = 0;
while i <= 1
    Smoothed_Conversion = smoothdata(Smoothed_Conversion);
    i = i+1;
end
%p_3 = polyfit(filteredData(:,2),Conversion,30);
%f1 = polyval(p_3,filteredData(:,2));
g_force = ((Smoothed_Conversion+120).*32.174)./(Smoothed_Conversion.*32.174);
subplot(2,2,1)
hold on
plot(TimeData,Smoothed_Data_Real)
%plot(TimeData,StrainGaugeData)
title('Time vs Signal')
ylabel('Signal')
xlabel('Time')
subplot(2,2,2)
hold on
plot(TimeData,Smoothed_Conversion)
%plot(Time,Conversion)
title('Time vs Mass(lbs)')
ylabel('Mass (lbs)')
xlabel('Time')
subplot(2,2,3:4)
plot(TimeData,g_force)
title('Time vs Vertical Force(Gs)')
ylabel('Force (Gs)')
xlabel('Time')

Smoothed_Data_MovMean = smoothdata(StrainGaugeData, 'movmean', 5);
Smoothed_Data_Loess = smoothdata(StrainGaugeData, 'loess', 20);
Smoothed_Data_RL = smoothdata(StrainGaugeData, 'rlowess', 20);

figure(5);
hold on
plot(TimeData, StrainGaugeData, 'k-', ...
     TimeData, Smoothed_Data_MovMean, 'b-', ...
     TimeData, Smoothed_Data_Loess, 'r-', ...
     TimeData, Smoothed_Data_RL, 'g-');
legend('Raw Data', 'MovMean', 'Loess', 'RLowess');
title('Comparison of Smoothing Methods');
ylabel('Signal');
xlabel('Time');
