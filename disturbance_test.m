clear; clc; close all;

% Settings
Kc   = 5;
tauI = 1;
tauD = 0.1;
tauF = 0.05;
Tsp  = 355;
Tc0  = 300;

x0 = [0.5; 350];
tspan = [0 10];
opts = odeset('MaxStep', 0.01);

[tP,   xP]   = ode45(@(t,x) cstr_p(t, x, Kc, Tsp, Tc0), tspan, x0, opts);
[tPI,  xPI]  = ode45(@(t,x) cstr_pi(t, x, Kc, tauI, Tsp, Tc0), tspan, [x0; 0], opts);
[tPID, xPID] = ode45(@(t,x) cstr_pid(t, x, Kc, tauI, tauD, tauF, Tsp, Tc0), tspan, [x0; 0; 350], opts);

% Plot
figure;
plot(tP, xP(:,2), 'b', tPI, xPI(:,2), 'g', tPID, xPID(:,2), 'm', 'LineWidth', 1.5); hold on;
yline(Tsp, 'r--', 'Setpoint');
xline(5, 'k:', 'Disturbance');
xlabel('Time (min)');
ylabel('Temperature (K)');
legend('P', 'PI', 'PID', 'Location', 'southeast');
grid on;
saveas(gcf, 'disturbance_test.png');
% Feed temperature: 350 K, drops to 340 K at t = 5 min
function Tf = feedtemp(t)
Tf = 350;
if t >= 5
    Tf = 340;
end
end

% Local functions (keep them at the end of the file)
function dxdt = cstr_p(t, x, Kc, Tsp, Tc0)
Tc = Tc0 + Kc*(Tsp - x(2));
Tc = min(max(Tc, 250), 350);
dxdt = cstr_model_d(t, x, Tc, feedtemp(t));
end

function dxdt = cstr_pi(t, x, Kc, tauI, Tsp, Tc0)
e  = Tsp - x(2);
Tc = Tc0 + Kc*(e + x(3)/tauI);
Tc = min(max(Tc, 250), 350);
dxdt = [cstr_model_d(t, x(1:2), Tc, feedtemp(t)); e];
end

function dxdt = cstr_pid(t, x, Kc, tauI, tauD, tauF, Tsp, Tc0)
e    = Tsp - x(2);
dTdt = (x(2) - x(4))/tauF;
Tc   = Tc0 + Kc*(e + x(3)/tauI - tauD*dTdt);
Tc   = min(max(Tc, 250), 350);
dxdt = [cstr_model_d(t, x(1:2), Tc, feedtemp(t)); e; dTdt];
end