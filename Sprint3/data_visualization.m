clear all
clc
close all

data = readtable('sim_output.csv'); 
time = data.time;                     
velocity_ms = data.velocity;          % m/s 
velocity_kmh = velocity_ms * 3.6;    % convert to km/h

% ---acceleration ---
acceleration = diff(velocity_ms) ./ diff(time); % m/s^2
time_acc = time(1:end-1) + diff(time)/2;      

% --- Plot velocity ---
figure;
plot(time, velocity_kmh, 'b', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Velocity (km/h)');
title('Cruise Control Simulation - Velocity');

S = stepinfo(velocity_kmh, time);
disp(S);

