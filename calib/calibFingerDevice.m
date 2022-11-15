%% calibrate finger devices
% load .dat file and create .txt calibration file for finger devices
% need to hard-code file names
% field names might also need to be adjusted properly depending on .dat
% file. 
% .dat file is assumed to have the following data fields;
%   kg: kg of weights used for calibration
%   Th: voltage values for thumb
%   In: voltage values for index finger
%   Mi: voltage values for middle finger
%   Ri: voltage values for ring finger
%   Li: voltage values for little finger
% 
% a-yokoi (2015 Dec)

%% load calib data file
%calibdatafile = 'C:\robot\calib\pneumatic_forcesensor_calib.dat';
%calibdatafile = 'Flatbox1_highforce_RIGHT_19-Jun-2017.dat';
%calibdatafile = 'LEFT_lowForce_FlatBox2_24-Jan-2018.dat';
%calibdatafile = 'flatbox3_highforce2_RIGHT_28-Mar-2018.dat';
%calibdatafile = 'flatbox3_highforce2_LEFT_28-Mar-2018.dat';
%calibdatafile = 'flatbox2_highforce2_LEFT_27-May-2018.dat';
%calibdatafile = 'LEFT_lowForce_FlatBox2_24-Jan-2018.dat';
%calibdatafile = 'Flatbox1_lowforce_RIGHT_02-Dec-2021.dat';
%calibdatafile = 'Flatbox1_lowforce_LEFT_02-Dec-2021.dat';
%calibdatafile = 'Flatbox1_highforce2_RIGHT_03-Dec-2021.dat';
calibdatafile = 'Flatbox1_highforce2_LEFT_12-Feb-2022.dat';
%calibdatafile = 'Flatbox1_highforce2_LEFT_03-Dec-2021.dat';
D = dload(calibdatafile);

%% regress with/without intercept (N=Vb)
% kg->Newton
N = D.kg*9.8;

% thumb
th = (D.Th'*D.Th)\(D.Th'*N);
% index
idx = (D.In'*D.In)\(D.In'*N);
% middle
mid = (D.Mi'*D.Mi)\(D.Mi'*N);
% ring
rin = (D.Ri'*D.Ri)\(D.Ri'*N);
% little
lit = (D.Li'*D.Li)\(D.Li'*N);

%% write 
%calibfile = sprintf('C:/robot/calib/flatbox2_highforce_RIGHT_%s.txt',date);
%calibfile = sprintf('flatbox1_highforce_RIGHT_%s.txt',date);
%calibfile = sprintf('LEFT_lowForce_FlatBox2_24--2017.txt');
%calibfile = 'flatbox3_highforce2_RIGHT_28-Mar-2018.txt';
%calibfile = 'flatbox3_highforce2_LEFT_28-Mar-2018.txt';
%calibfile = 'flatbox2_highforce2_LEFT_27-May-2018.txt';
%calibfile = 'LEFT_lowForce_FlatBox2_24-Jan-2018.txt';
%calibfile = 'Flatbox1_lowforce_RIGHT_02-Dec-2021.txt';
%calibfile = 'Flatbox1_lowforce_LEFT_02-Dec-2021.txt';
%calibfile = 'Flatbox1_highforce2_RIGHT_03-Dec-2021.txt';
calibfile = 'Flatbox1_highforce2_LEFT_12-Feb-2022.txt';
%calibfile = 'Flatbox1_highforce2_LEFT_03-Dec-2021.txt';
dlmwrite(calibfile,[th(1),idx(1),mid(1),rin(1),lit(1)],'\t')

%% plot
P.N = repmat(N,5,1);
P.V = [D.Th;D.In;D.Mi;D.Ri;D.Li];
P.finger = [ones(size(N));...
            2*ones(size(N));...
            3*ones(size(N));...
            4*ones(size(N));...
            5*ones(size(N))];
figure;
scatterplot(P.V,P.N,'split',P.finger,...
            'leg',{'Thumb','Index','Middle','Ring','Little'});
ylabel('Newton');xlabel('Voltage')
                