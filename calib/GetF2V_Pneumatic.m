%% get force to volt coefficients
% 
% calculate force to voltage coefficients for pneumatic device
% need to hard-code file names
% field names might also need to be adjusted properly depending on .dat
% file. 
% .dat file is assumed to have the following data fields;
%   amplitude: voltage values given in TestPneumaticDevice.exe
%   PeakForceT: force values (gBox[0].getForce(0)) for thumb
%   PeakForceI: force values (gBox[0].getForce(1)) for index finger
%   PeakForceM: force values (gBox[0].getForce(2)) for middle finger
%   PeakForceR: force values (gBox[0].getForce(3)) for ring finger
%   PeakForceL: force values (gBox[0].getForce(4)) for little finger
% 
% a-yokoi (2015 Dec)

%% with/without intercept
type = 'w_intercept'; % 'wo_intercept';

%% load calib data file
%D = dload('C:\data\TestDevice\TestPneumaticDevice\TestPneumaticDevice_step.dat');
D = dload('C:\data\TestDevice\TestPneumaticDevice\TestPneumaticDevice_cal2016.dat');

%% regress with/without intercept (V=Fb)
fields = {'PeakForceT','PeakForceI','PeakForceM','PeakForceR','PeakForceL'};
force = []; % for plotting
for f=1:5
   d = getrow(D,D.finger==f);
   V = d.amplitude;
   F = d.(fields{f});
   switch (type)
       case 'w_intercept'
           F = [F,ones(size(F))]; % intercept
   end
   B = (F'*F)\(F'*V);
   Coeff(f) = B(1);
   
   index = find(D.finger==f);
   force(index,1) = F(:,1);
end
D.force = force; clear force
%% write 
%calibfile = sprintf('C:/robot/calib/right_lowforce_pneumatic_f2v_%s.txt',date);
calibfile = sprintf('C:/robot/calib/right_highforce_pneumatic_f2v_%s_%s.txt',type,date);
dlmwrite(calibfile,Coeff,'\t');

%% plot
figure;
[~,b]=scatterplot(D.force,D.amplitude,'split',D.finger,...
            'regression','leastsquare','printcorr',...
            'leg',{'Thumb','Index','Middle','Ring','Little'});
xlabel('Newton');ylabel('Voltage');axis square
set(gca,'xlim',[0 10],'ylim',[0 10]);
              

