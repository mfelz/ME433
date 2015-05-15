Q = csvread('C:\Users\Matt\Documents\ME433\hidapi-master\windows\accs.txt');
z = Q(:,1);
zMAF = Q(:,2);
zFIR = Q(:,3);
time = 20/500:20/500:20;

% Choose filter cutoff frequency (6 kHz)
cutoff_hz = 1;
 
% Normalize cutoff frequency (wrt Nyquist frequency)
nyq_freq = 25 / 2;
cutoff_norm = cutoff_hz / nyq_freq;
% FIR filter order (i.e. number of coefficients - 1)
order = 12;
fir_coeff = fir1(order, cutoff_norm);
% Filter the signal with the FIR filter
zFIR= filter(fir_coeff, 1, z);

%MAF FILTER
b = (1/5)*ones(1,5);
a = 1;
y = filter(b,a,z);

subplot(2,1,1)

plot(time,z,time,zMAF,time,zFIR)
title('Acceleration in the Z direction vs time');
xlabel('time (sec)')
ylabel('acceleration (mm/s^2)');
legend('unfiltered data', 'MAF Filter', 'FIR Filter');
ylim([-20000, -10000]);

Fs = 25; % Sampling frequency
nfft =512; % Next power of 2 from length of y; % Length of FFT
Z = fft(z,nfft);
ZMAF = fft(zMAF,nfft);
ZFIR = fft(zFIR,nfft);
% FFT is symmetric, throw away second half
Z = Z(1:nfft/2-1);
ZMAF = ZMAF(1:nfft/2-1);
ZFIR = ZFIR(1:nfft/2-1);
% Take the magnitude of fft of x
mZ = abs(Z);
mZMAF = abs(ZMAF);
mZFIR = abs(ZFIR);
% Frequency vector
f = (1:nfft/2-1)*Fs/nfft;

subplot(2,1,2)
plot(f,mZ,f,mZMAF, f, mZFIR);
%%line([1, 1], [ 0 5e5], 'color' , 'k', 'LineStyle', '--');
title('Frequency response of accelerations');
ylabel('Magnitude')
xlabel('freq(hz)');
legend('unfiltered data', 'MAF Filter', 'FIR Filter');
ylim([0 4e5]);


