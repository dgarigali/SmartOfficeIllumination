%% Initial Code

clc
clear all

% Constants
measure_num = 300;
steps_num = 5;

% Open serial connection
s = serial('COM5');
set(s ,'BaudRate', 2000000);
fopen(s);
pause(2);

% Receive static gain
Go = str2double(fscanf(s));

%% Positive and negative steps

% Illuminance in each step
illuminance_array = zeros(1, steps_num * 2);
delta_u_array = zeros(1, steps_num * 2);

for i=1:steps_num
    illuminance_array(i) = i*Go;
    delta_u_array(i) = 1;
end

for i=1:steps_num
    illuminance_array(steps_num + i) = (steps_num-i)*Go;
    delta_u_array(steps_num + i) = -1;
end

% Create time and voltage arrays
time_array = zeros(1, measure_num*steps_num*2);
voltage_array = zeros(1, measure_num*steps_num*2);

% Read measurements through serial
for i=1:steps_num*2
    for j=1:measure_num
        time_array(measure_num*(i-1)+j) = str2double(fscanf(s));
        voltage_array(measure_num*(i-1)+j) = str2double(fscanf(s));
    end
end

% Close connection
fclose(s);
delete(s);
clear s;

% Plot
time_array = time_array - time_array(1);
figure(1)
plot(time_array, voltage_array);
xlabel('Time[us]');
ylabel('Voltage[V]');
title('LDR step response');
hold on;
load('u_step.mat');
plot(time_array, u_array);
legend('v(t)','u(t)');




% Create array with 63% (positive step) and 37% (negative step) of last step value
step_voltage_array = zeros(steps_num, measure_num*steps_num*2);
intersection_array = zeros(1, steps_num*2);
taus_array = zeros(1, steps_num*2);
delta_v_array = zeros(1, steps_num*2);

% Calculate time constant for each positive step
for i=1:steps_num
    step_voltage_array(i,:) = 0.63*(voltage_array(measure_num*i)-voltage_array(1 + measure_num*(i-1))) + voltage_array(1 + measure_num*(i-1));
    plot(time_array, step_voltage_array(i,:));
    p = InterX([time_array; voltage_array],[time_array; step_voltage_array(i,:)]);
    intersection_array(i) = p(1,1);
    taus_array(i) = intersection_array(i) - time_array(1 + measure_num*(i-1));
    delta_v_array(i) = voltage_array(measure_num*i) - voltage_array(1 + measure_num*(i-1));
end

% Calculate time constant for each negative step
for i=steps_num+1:steps_num*2
    step_voltage_array(i,:) = 0.37*(voltage_array(1 + measure_num*(i-1)) - voltage_array(measure_num*i)) + voltage_array(measure_num*i);
    plot(time_array, step_voltage_array(i,:));
    p = InterX([time_array; voltage_array],[time_array; step_voltage_array(i,:)]);
    intersection_array(i) = p(1,2);
    taus_array(i) = intersection_array(i) - time_array(1 + measure_num*(i-1));
    delta_v_array(i) = voltage_array(measure_num*i) - voltage_array(1 + measure_num*(i-1));
end

% Plot time constant in function of illuminance
figure(2)
stem(illuminance_array, taus_array);
hold on;
xlabel('x [LUX]');
ylabel('tau [us]');
title('Variation of time constant with desired illuminance');

% Calculate Ko
Ko_array = zeros(1, steps_num*2);
for i=1:steps_num*2
    Ko_array(i) = delta_v_array(i)/delta_u_array(i);
end

%% Curve fit

% Positive Step
[P1,~] = polyfit(illuminance_array(1:steps_num),taus_array(1:steps_num),4);
x =linspace(illuminance_array(1), illuminance_array(steps_num),100);
y = polyval(P1,x);
plot(x,y);
hold on;


% Negative Step
[P2,~] = polyfit(illuminance_array(steps_num+1:steps_num*2),taus_array(steps_num+1:steps_num*2),4);
x =linspace(illuminance_array(steps_num+1), illuminance_array(steps_num*2),100);
y = polyval(P2,x);
plot(x,y);
hold on;

% Average of both steps
illuminance_array_avg = [illuminance_array(10), illuminance_array(1:5)];
taus_array_avg = [taus_array(10), (taus_array(1)+taus_array(9))/2, (taus_array(2)+taus_array(8))/2, (taus_array(3)+taus_array(7))/2, (taus_array(4)+taus_array(6))/2, taus_array(5)];
stem(illuminance_array_avg, taus_array_avg);
hold on;
[P3,~] = polyfit(illuminance_array_avg,taus_array_avg,5);
x =linspace(illuminance_array_avg(1), illuminance_array_avg(steps_num+1),100);
y = polyval(P3,x);
plot(x,y);
legend('taus','positive steps curve fit', 'negative steps curve fit', 'average taus', 'average curve fit');

% Calculate PI parameters
for i=1:steps_num*2
    sys = tf ([Ko_array(i)],[taus_array(i)/1000000 1]);
    [C_pi,info] = pidtune(sys,'PI');
    disp(C_pi);
end