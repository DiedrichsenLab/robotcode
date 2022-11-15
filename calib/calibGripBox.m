%% Calibrate Grip Box  (slip device)
% Creates calibration file for grip box device.
% Loads .dat file and creates .txt calibration file for finger devices.
%
% Need to harcode file name for input calibration datafile 
% Ouput filename is "gripBox_DATE" (Date is dd-MON-yyyy). 
% PLEASE DON't UNNECESSARILY ADJUST OUTPUT FILENAME- aim to be consistent
%
% .dat file is assumed to have the following data fields;
%   kg: kg of weights used for calibration
%   Fs1: voltage values for thumb (for force sensor nearest palm)
%   Fs2: voltage values for index finger (for force sensor nearest palm)
%   Fs3: voltage values for middle finger (for force sensor nearest palm)
%   Fs4: voltage values for ring finger (for force sensor nearest palm)
%   Fs5: voltage values for little finger (for force sensor nearest palm)
%   Fs6: voltage values for thumb (for force sensor furthest from palm)
%   Fs7: voltage values for index finger (for force sensor furthest from palm)
%   Fs8: voltage values for middle finger (for force sensor furthest from palm)
%   Fs9: voltage values for ring finger (for force sensor furthest from palm)
%   Fs10: voltage values for little finger (for force sensor furthest from palm)
% 
% SArbuckle (2016); based on calibFingerDevice.m by AYokoi

% % Load calib data file
inname = '/Volumes/MotorControl/robotcode/calib/gripBox_calibdata_file.dat'; % calib data file
D = dload(inname);

% % Regress 
N = D.kg*9.8;

for q = 1:10 % for each force sensor
    eval(sprintf('Fs%d = (D.Fs%d''* D.Fs%d)\\(D.Fs%d''* N);',q,q,q,q));
end
% above regression equation: Scaleparam = (D.Fsx'*D.Fsx)\(D.Fsx'*N);

% % wrtie/save output (default saves output filename to same path as input calib data file)
pathstr = fileparts(inname);
outname = fullfile(pathstr,sprintf('gripBox_%s.txt',date)); 
dlmwrite(outname,[Fs1(1),Fs2(1),Fs3(1),Fs4(1),Fs5(1),...
                  Fs6(1),Fs7(1),Fs8(1),Fs9(1),Fs10(1)],'\t');
              
              
% % plot scaling params
P.N = repmat(N,10,1);
P.V = [D.Fs1;D.Fs2;D.Fs3;D.Fs4;D.Fs5;D.Fs6;D.Fs7;D.Fs8;D.Fs9;D.Fs10];
finger = [ones(size(N));...
            2*ones(size(N));...
            3*ones(size(N));...
            4*ones(size(N));...
            5*ones(size(N))];
P.finger = [finger;finger+5];
figure;
scatterplot(P.V,P.N,'split',P.finger,...
            'leg',{'Th1','In1','Mi1','Ri1','Li1','Th2','In2','Mi2','Ri2','Li2'});
ylabel('Newton');xlabel('Voltage')