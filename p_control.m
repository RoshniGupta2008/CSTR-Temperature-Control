clear; clc; close all;

% Controller settings
Kc  = 5;        % proportional gain
Tsp = 355;      % setpoint (K)
Tc0 = 300;      % base coolant temperature (K)

% Simulation
x0 = [0.5; 350];
tspan = [0 10];
[t, x] = ode45(@(t,x) cstr_p(t, x, Kc, Tsp, Tc0), tspan, x0);

% Coolant temperature used by the controller
Tc = Tc0 + Kc*(Tsp - x(:,2));
Tc = min(max(Tc, 250), 350);

% Plot
figure;
subplot(2,1,1);
plot(t, x(:,2), 'b', 'LineWidth', 1.5); hold on;
yline(Tsp, 'r--', 'Setpoint');
ylabel('Temperature (K)');
grid on;

subplot(2,1,2);
plot(t, Tc, 'k', 'LineWidth', 1.5);
xlabel('Time (min)');
ylabel('Coolant temp Tc (K)');
grid on;
saveas(gcf, 'p_control.png');
% Closed-loop function (local function, keep it at the end of the file)
function dxdt = cstr_p(t, x, Kc, Tsp, Tc0)
Tc = Tc0 + Kc*(Tsp - x(2));
Tc = min(max(Tc, 250), 350);   % coolant limits
dxdt = cstr_model(t, x, Tc);
end