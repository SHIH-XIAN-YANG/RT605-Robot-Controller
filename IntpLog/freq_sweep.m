%%
clc;
clear all;
close all;
%%
% Load data from CSV
data = readmatrix('sweep_0_2024_10_21_14_54_3.csv'); % Adjust the path and skip header row if needed
time = data(:, 1);
signal_ref = data(:, 8);
signal = data(:, 14);

% shift = signal_ref(1);
% 
% for i=1:length(signal_ref)
%     signal_ref(i) = signal_ref(i)-shift;
%     signal(i) = signal(i)-shift;
% end

ts = 0.0005;
fs = 1/ts;
sz = size(signal_ref);
n = 0;
if sz(1) > 1
    n = sz(1);
else
    n = sz(2);
end

% fft:
xr_fft = fft(signal_ref);
x_fft = fft(signal);

xr_fft = xr_fft(1:fix(n/2));
x_fft = x_fft(1:fix(n/2));

% frequency response:
freq_step = fs/n;
freq_axis = 0.0:freq_step:(fs/2)-freq_step;
res = (x_fft./xr_fft);
mag = 20.0*log10(abs(res));
phase = rad2deg(atan2(imag(res), real(res)));


% joint Plot
figure();
plot(time, signal);
title('joint data');
xlabel('time');
ylabel('degree()');
hold on;
plot(time, signal_ref);
legend('ref','act')
legend();
grid on;

% Bode plot:
figure()
subplot(2, 1, 1)
semilogx(freq_axis, mag);
title("$Magnitude$", "Interpreter", "Latex");
xlabel("$(Hz)$", "Interpreter", "Latex"); ylabel("$dB$", "Interpreter", "Latex");
grid on;
xlim([0, inf]);

subplot(2, 1, 2)
semilogx(freq_axis, phase);
title("$Phase$", "Interpreter", "Latex");
xlabel("$(Hz)$", "Interpreter", "Latex"); ylabel("$degree$", "Interpreter", "Latex");
grid on;
xlim([0, inf]);

% compute bandwidth and show it:
BW_pos = 0;
index = 1;
for i = 1:length(mag)
    if (-3*0.95) > (mag(i)) && (mag(i)) > (-3*1.05) 
        BW_pos = freq_axis(i);
        break;
    end
    index = index+1;
end

% tag BW:
subplot(2, 1, 1)
hold on;
bw = mag(index);
bw_line = min(mag):0.05:bw;
plot(BW_pos*ones(1, length(bw_line)), bw_line);
str = sprintf("BW = %f", BW_pos);
text(BW_pos, 0.5*bw, str);
hold off; 
