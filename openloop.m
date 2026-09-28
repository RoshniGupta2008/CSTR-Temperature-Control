clear; clc; close all;

% Initial steady state
x0 = [0.5; 350];        % [CA (mol/L); T (K)]
tspan = [0 10];         % time (min)

% Case 1: coolant stays at 300 K (steady state)
[t1, x1] = ode45(@(t,x) cstr_model(t, x, 300), tspan, x0);

% Case 2: coolant steps up to 305 K
[t2, x2] = ode45(@(t,x) cstr_model(t, x, 305), tspan, x0);

% Plot
figure;
subplot(2,1,1);
plot(t1, x1(:,2), 'b', t2, x2(:,2), 'r', 'LineWidth', 1.5);
ylabel('Temperature (K)');
legend('Tc = 300 K', 'Tc = 305 K');
grid on;

subplot(2,1,2);
plot(t1, x1(:,1), 'b', t2, x2(:,1), 'r', 'LineWidth', 1.5);
xlabel('Time (min)');
ylabel('CA (mol/L)');
grid on;
saveas(gcf, 'open_loop.png');