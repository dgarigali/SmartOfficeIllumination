%% Initial Code

clc
clear all

% Constants
measure_num = 300;
steps_num = 1;

m = -0.6505;
b = 4.778;

% Open serial connection
s = serial('COM5');
set(s ,'BaudRate', 2000000);
fopen(s);
pause(2);

% Receive static gain
Go = str2double(fscanf(s));

%% Positive Step

% Illuminance in each step
illuminance_array = zeros(1, steps_num);
delta_u_array = zeros(1, steps_num);
for i=1:steps_num
    illuminance_array(i) = i*Go;
    delta_u_array(i) = 1;
end

% Create time and voltage arrays
time_array = zeros(1, measure_num*steps_num);
voltage_array = zeros(1, measure_num*steps_num);

% Read measurements through serial
for i=1:steps_num
    for j=1:measure_num
        time_array(measure_num*(i-1)+j) = str2double(fscanf(s));
        voltage_array(measure_num*(i-1)+j) = str2double(fscanf(s));
    end
end

% Close connection
fclose(s);
delete(s);
clear s;

% Convert time from microseconds to seconds
%time_array = time_array/1000000;
    
% Plot
figure(1)
plot(time_array, voltage_array);
hold on;

% Create array with 63% of last step value
step_voltage_array = zeros(steps_num, measure_num*steps_num);
intersection_array = zeros(1, steps_num);
taus_array = zeros(1, steps_num);
delta_v_array = zeros(1, steps_num);

% Calculate time constant for each step
for i=1:steps_num
    step_voltage_array(i,:) = 0.63*(voltage_array(measure_num*i)-voltage_array(1 + measure_num*(i-1))) + voltage_array(1 + measure_num*(i-1));
    plot(time_array, step_voltage_array(i,:));
    p = InterX([time_array; voltage_array],[time_array; step_voltage_array(i,:)]);
    intersection_array(i) = p(1);
    taus_array(i) = intersection_array(i) - time_array(1 + measure_num*(i-1));
    delta_v_array(i) = voltage_array(measure_num*i) - voltage_array(1 + measure_num*(i-1));
end

% Plot time constant in function of illuminance
figure(2)
stem(illuminance_array, taus_array);
hold on;

%% Curve fit
[P,S] = polyfit(illuminance_array,taus_array,4);
x=linspace(illuminance_array(1), illuminance_array(steps_num),100);
y = polyval(P,x);
plot(x,y);

% Calculate Ko
Ko_array = zeros(1, steps_num);
for i=1:steps_num
    Ko_array(i) = delta_v_array(i)/delta_u_array(i);
end

% Calculate PI parameters
for i=1:steps_num
    sys = tf ([Ko_array(i)],[taus_array(i)/1000000 1]);
    [C_pi,info] = pidtune(sys,'PI');
    disp(C_pi);
end

% Vi-y plot
y_array = zeros(1, measure_num*steps_num);
for i=1:measure_num*steps_num
   y_array(i) = 10.^((log10((10000*5/voltage_array(i))-10000)-b)/m);
end
figure(3)
plot(voltage_array, y_array);
xlabel('v [V]');
ylabel('y [LUX]');
title('Inverse LDR function');