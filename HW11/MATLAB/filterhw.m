%% Preprocessing
% Path of the accdata please change this corresponding, thank you!
clc;
clear;
addpath('C:\Users\sunyue\Desktop\Advanced Mechatronic\HW11\dataset')

%% Processing 1Hz vibration data
% Reading the 1hz dataset
[zraw,zMAF,zFIR] = textread('data1hz.txt', '%f%f%f','commentstyle', 'shell');
zraw = zraw(200:600);
zMAF = zMAF(200:600);
zFIR = zFIR(200:600);

% Plotting the data
figure(1)
subplot(3,1,1);
plot(zraw(100:350),'k','LineWidth',2);
axis([0 250 -13 -6])
grid on
title('1Hz Vibration Raw Data')
subplot(3,1,2);
plot(zMAF(100:350),'r','LineWidth',2);
axis([0 250 -13 -6])
grid on
title('1Hz Vibration with MAF')
subplot(3,1,3);
plot(zFIR(100:350),'b','LineWidth',2);
axis([0 250 -13 -6])
grid on
title('1Hz Vibration with FIR')

% Plotting the FFT distribution
Fs = 25;                    % Sampling frequency
T = 1/Fs;                     % Sample time
L = 400;                     % Length of signal
t = (0:L-1)*T;                % Time vector
NFFT = 2^nextpow2(L); % Next power of 2 from length of y
fftzraw = fft(zraw,NFFT)/L;
fftzMAF = fft(zMAF,NFFT)/L;
fftzFIR = fft(zFIR,NFFT)/L;
f = Fs/2*linspace(0,1,NFFT/2+1);

figure(2)
subplot(1,3,1);
stem(f,2*abs(fftzraw(1:NFFT/2+1)),'r','LineWidth',1.5)
axis([0 12 0 1.6])
title('1Hz Vibration Raw Data FFT')
grid on
subplot(1,3,2);
stem(f,2*abs(fftzMAF(1:NFFT/2+1)),'k','LineWidth',1.5)
axis([0 12 0 1.6])
title('1Hz Vibration with MAF FFT')
grid on
subplot(1,3,3);
stem(f,2*abs(fftzFIR(1:NFFT/2+1)),'b','LineWidth',1.5)
axis([0 12 0 1.6])
title('1Hz Vibration with FIR FFT')
grid on

%% Processing no vibration data
% Reading the 1hz dataset
[zraw2,zMAF2,zFIR2] = textread('datano.txt', '%f%f%f','commentstyle', 'shell');
zraw2 = zraw2(100:500);
zMAF2 = zMAF2(100:500);
zFIR2 = zFIR2(100:500);

% Plotting the data
figure(3)
subplot(3,1,1);
plot(zraw2(100:350),'k','LineWidth',2);
axis([0 250 -10 -9.4])
grid on
title('No Vibration Raw Data')
subplot(3,1,2);
plot(zMAF2(100:350),'r','LineWidth',2);
axis([0 250 -10 -9.4])
grid on
title('No Vibration with MAF')
subplot(3,1,3);
plot(zFIR2(100:350),'b','LineWidth',2);
axis([0 250 -10 -9.4])
grid on
title('No Vibration with FIR')

% Plotting the FFT distribution
Fs = 25;                    % Sampling frequency
T = 1/Fs;                     % Sample time
L = 400;                     % Length of signal
t = (0:L-1)*T;                % Time vector
NFFT = 2^nextpow2(L); % Next power of 2 from length of y
fftzraw2 = fft(zraw2,NFFT)/L;
fftzMAF2 = fft(zMAF2,NFFT)/L;
fftzFIR2 = fft(zFIR2,NFFT)/L;
f = Fs/2*linspace(0,1,NFFT/2+1);

figure(4)
subplot(1,3,1);
stem(f,2*abs(fftzraw2(1:NFFT/2+1)),'r','LineWidth',1.5)
axis([0 12 0 1.6])
title('No Vibration Raw Data FFT')
grid on
subplot(1,3,2);
stem(f,2*abs(fftzMAF2(1:NFFT/2+1)),'k','LineWidth',1.5)
axis([0 12 0 1.6])
title('No Vibration with MAF FFT')
grid on
subplot(1,3,3);
stem(f,2*abs(fftzFIR2(1:NFFT/2+1)),'b','LineWidth',1.5)
axis([0 12 0 1.6])
title('No Vibration with FIR FFT')
grid on