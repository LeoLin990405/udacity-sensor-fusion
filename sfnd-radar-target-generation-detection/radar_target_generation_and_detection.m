%% Radar Target Generation and Detection
% Sensor Fusion Nanodegree radar project implementation.

clear;
clc;
close all;

%% User-defined range and velocity of target
target_range = 110;
target_velocity = -20;

%% FMCW waveform design
c = 3e8;
max_range = 200;
range_resolution = 1;
max_velocity = 100;

fc = 77e9;
B = c / (2 * range_resolution);
Tchirp = 5.5 * 2 * max_range / c;
slope = B / Tchirp;

fprintf('Bandwidth: %.3e Hz\\n', B);
fprintf('Chirp time: %.3e s\\n', Tchirp);
fprintf('Slope: %.3e Hz/s\\n', slope);

%% Operating parameters
Nd = 128;
Nr = 1024;
t = linspace(0, Nd * Tchirp, Nr * Nd);

Tx = zeros(1, length(t));
Rx = zeros(1, length(t));
Mix = zeros(1, length(t));
r_t = zeros(1, length(t));
td = zeros(1, length(t));

%% Signal generation and moving target simulation
for i = 1:length(t)
    r_t(i) = target_range + target_velocity * t(i);
    td(i) = 2 * r_t(i) / c;

    Tx(i) = cos(2 * pi * (fc * t(i) + slope * t(i)^2 / 2));
    delayed_time = t(i) - td(i);
    Rx(i) = cos(2 * pi * (fc * delayed_time + slope * delayed_time^2 / 2));
    Mix(i) = Tx(i) .* Rx(i);
end

%% Range FFT
Mix = reshape(Mix, [Nr, Nd]);
sig_fft = fft(Mix, Nr);
sig_fft = abs(sig_fft ./ Nr);
sig_fft = sig_fft(1:Nr / 2, 1);

figure('Name', 'Range from First FFT');
plot(sig_fft);
axis([0 200 0 1]);
title('Range from First FFT');
xlabel('Range (m)');
ylabel('Normalized amplitude');

%% 2D FFT for range-Doppler map
sig_fft2 = fft2(Mix, Nr, Nd);
sig_fft2 = sig_fft2(1:Nr / 2, 1:Nd);
sig_fft2 = fftshift(sig_fft2);
RDM = abs(sig_fft2);
RDM = 10 * log10(RDM);

doppler_axis = linspace(-100, 100, Nd);
range_axis = linspace(-200, 200, Nr / 2) * ((Nr / 2) / 400);

figure('Name', 'Range Doppler Map');
surf(doppler_axis, range_axis, RDM);
title('Range Doppler Map');
xlabel('Velocity (m/s)');
ylabel('Range (m)');
zlabel('Amplitude (dB)');

%% 2D CFAR
Tr = 10;
Td = 8;
Gr = 4;
Gd = 4;
offset = 6;

cfar_output = zeros(size(RDM));

for i = Tr + Gr + 1 : (Nr / 2) - (Tr + Gr)
    for j = Td + Gd + 1 : Nd - (Td + Gd)
        noise_level = 0;
        training_cells = 0;

        for p = i - (Tr + Gr) : i + (Tr + Gr)
            for q = j - (Td + Gd) : j + (Td + Gd)
                in_guard_or_cut = abs(i - p) <= Gr && abs(j - q) <= Gd;
                if ~in_guard_or_cut
                    noise_level = noise_level + db2pow(RDM(p, q));
                    training_cells = training_cells + 1;
                end
            end
        end

        threshold = pow2db(noise_level / training_cells) + offset;
        if RDM(i, j) > threshold
            cfar_output(i, j) = 1;
        end
    end
end

figure('Name', '2D CFAR Output');
surf(doppler_axis, range_axis, cfar_output);
title('2D CFAR Output');
xlabel('Velocity (m/s)');
ylabel('Range (m)');
zlabel('Detection');
