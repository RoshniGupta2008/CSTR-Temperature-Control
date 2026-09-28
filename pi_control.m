clear; clc; close all;

% Settings
Kc   = 5;        % proportional gain
tauI = 1;        % integral time (min)
Tsp  = 355;      % setpoint (K)
Tc0  = 300;      % base coolant temperature (K)

x0 = [0.5; 350];
tspan = [0 10];

% P controller
[tP, xP] = ode45(@(t,x) cstr_p(t, x, Kc, Tsp, Tc0), tspan, x0);

% PI controller (third state = integral of error)
[tPI, xPI] = ode45(@(t,x) cstr_pi(t, x, Kc, tauI, Tsp, Tc0), tspan, [x0; 0]);

% Plot
figure;
plot(tP, xP(:,2), 'b', tPI, xPI(:,2), 'g', 'LineWidth', 1.5); hold on;
yline(Tsp, 'r--', 'Setpoint');
xlabel('Time (min)');
ylabel('Temperature (K)');
legend('P', 'PI', 'Location', 'southeast');
grid on;
saveas(gcf, 'pi_control.png');

% Local functions (keep them at the end of the file)
function dxdt = cstr_p(t, x, Kc, Tsp, Tc0)
Tc = Tc0 + Kc*(Tsp - x(2));
Tc = min(max(Tc, 250), 350);
dxdt = cstr_model(t, x, Tc);
end

function dxdt = cstr_pi(t, x, Kc, tauI, Tsp, Tc0)
e  = Tsp - x(2);
Tc = Tc0 + Kc*(e + x(3)/tauI);
Tc = min(max(Tc, 250), 350);
dxdt = [cstr_model(t, x(1:2), Tc); e];   % third state = integral of e
end