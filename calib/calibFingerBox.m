%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calib finger box
%
%
%
%

clear;
cd 'C:\robot\calib';
% const
g = 9.81; % ([N/(m/sec)])

% load data
load('FlatKeyboard_2015.mat');

% define model
f = fittype('a*x'); % ignore intercept

%% get slope
y = g*calib.w_g'/1000; % translate grams into Newtons
for ch = 1:5
    % set x values
    xL = calib.L_mV.(['ch',num2str(ch-1)])';
    xL = xL - xL(1); % subtract offset
    xR = calib.R_mV.(['ch',num2str(ch-1)])';
    xR = xR - xR(1); % subtract offset
    
    % fit model
    result = fit(xL,y,f);
    slope.L.(['ch',num2str(ch-1)]) = result.a;
    
    result = fit(xR,y,f);
    slope.R.(['ch',num2str(ch-1)]) = result.a;
end

% show slopes
slope.L
slope.R

% save into files
calibFileName = sprintf('flatbox2_highforce_RIGHT_%s.txt',date);
dsave(['left_',calibFileName],slope.L);
dsave(['right_',calibFileName],slope.R);
