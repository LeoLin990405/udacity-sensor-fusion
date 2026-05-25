# Radar Target Generation and Detection

This submission implements the Sensor Fusion radar target generation and detection project in MATLAB.

## FMCW Waveform Design

The waveform is designed from the project requirements:

- Maximum range: 200 m.
- Range resolution: 1 m.
- Maximum velocity: 100 m/s.
- Carrier frequency: 77 GHz.

The bandwidth is calculated as `B = c / (2 * range_resolution)`, resulting in `1.5e8 Hz`. The chirp time is `5.5 * 2 * max_range / c`, resulting in approximately `7.33e-6 s`. The chirp slope is `B / Tchirp`, which is approximately `2.045e13 Hz/s`, matching the expected rubric value around `2e13`.

## Simulation Loop

The script simulates a target with initial range `110 m` and velocity `-20 m/s`. For every timestamp, it updates the target range, calculates the time delay, generates the transmitted and received chirp signals, and multiplies them to create the beat signal.

## Range FFT

The beat signal is reshaped into the range-time grid and processed with a one-dimensional FFT. The result is normalized and plotted for the first chirp. The strongest peak appears near the configured target range of `110 m`, within the expected tolerance.

## 2D CFAR

The script computes a range-Doppler map using `fft2`, then applies 2D CFAR. I used:

- Training cells: `Tr = 10`, `Td = 8`.
- Guard cells: `Gr = 4`, `Gd = 4`.
- Threshold offset: `6 dB`.

For each cell under test, the algorithm converts surrounding training cells from dB to linear power, averages the noise estimate, converts the threshold back to dB, adds the offset, and compares the CUT against the threshold. Cells near the boundaries are left at zero because a full training and guard window is not available there.

The final CFAR map suppresses the noise floor and keeps the target detection in the expected range-Doppler region.
