clc; clear; close all

%% Choose Parsed File

filename = "004_endurance_parsed.csv";
opts = detectImportOptions(filename);
opts = setvartype(opts, 1,'char');
readMat = readtable(filename, opts);
readMat = table2cell(readMat);

%% Enter CAN IDs and column position for values to extract

CANID = [
    "500"
        ];

indexCAN = [
    1
           ];
timeCAN = 9;


%% Reading

%%% Reads all wanted data into cell array finalMat
IDMat = readMat(:,1);
finalMat = cell(length(CANID));

for i = 1:length(CANID)
   IDCurrent = cellfun(@(x) strcmp(x, CANID(i)), IDMat);

   if mean(IDCurrent) == 0
       warning('CANID: ['+ CANID(i)+ '] not detected');
   end

   currentMat = readMat(IDCurrent, 1:end);

   for j = 1:length(indexCAN)
       currentData = currentMat(:,indexCAN(j)+1);
       finalMat{i}(:,j) = currentData;
   end

   finalMat{i}(:,end+1) = currentMat(:,end);
   
end

%% Organizing 

close all;

%%% Strain Gauge Calibration Data
SGWeight = [0 5.2 10.2 20.2 29.4 38.2 29.4 20.2 10.2 5.1 0];
SGRead = [2962 2964 2969 2980 2990 3001 2989 2978 2966 2961 2955];
SGRead = SGRead - SGRead(1);
fitLine = polyfit(SGRead,SGWeight, 1);

figure;
hold on
LinFit = plot(SGRead, polyval(fitLine, SGRead));
scatter(SGRead, SGWeight);
legend(LinFit, ['Slope = ' , num2str(fitLine(1))])
xlabel("SGRead (Signal)")
ylabel("SGWeight (Weight in LBF)")


%%% Post-Processing and Filtering Data
SGData = finalMat{1};
SGData = cell2mat(SGData);
SGData(:,1) = SGData(:,1)  - 0 ; %mean(SGData(20:100,1)) ;
SGData(:,1) = SGData(:,1) .* 1; %((3.3* 7.4)/(4095 * 4.7));
filtSGData = smoothdata(SGData(:,1),"gaussian", 200);

figure;
hold on
plot(SGData(:,2), SGData(:,1));
plot(SGData(:,2), filtSGData);
xlabel("Time (s)")

figure;
hold on
Unfiltered = plot(SGData(:,2), polyval(fitLine, SGData(:,1))./600);
Filtered = plot(SGData(:,2), polyval(fitLine, filtSGData)./600);
xlabel("Time (s)")
ylabel("Force in G's on arm")
legend([Unfiltered, Filtered], ["Unfiltered", "Filtered"])














