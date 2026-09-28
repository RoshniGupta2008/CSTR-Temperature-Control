clear; clc; close all;

% Settings
Kc   = 5;
tauI = 1;
tauD = 0.1;
tauF = 0.05;
Tsp  = 355;
Tc0  = 300;
T0   = 350;

x0 = [0.5; 350];
tspan = [0 10];
opts = odeset('MaxStep', 0.01);

% Run the three controllers
[tP,   xP]   = ode45(@(t,x) cstr_p(t, x, Kc, Tsp, Tc0), tspan, x0, opts);
[tPI,  xPI]  = ode45(@(t,x) cstr_pi(t, x, Kc, tauI, Tsp, Tc0), tspan, [x0; 0], opts);
[tPID, xPID] = ode45(@(t,x) cstr_pid(t, x, Kc, tauI, tauD, tauF, Tsp, Tc0), tspan, [x0; 0; 350], opts);

t_all = {tP, tPI, tPID};
y_all = {xP(:,2), xPI(:,2), xPID(:,2)};
names = {'P'; 'PI'; 'PID'};

stepSize = Tsp - T0;   % 5 K

Overshoot_K    = zeros(3,1);
Overshoot_pct  = zeros(3,1);
RiseTime_min   = zeros(3,1);
SettlingTime_min = zeros(3,1);
SSError_K      = zeros(3,1);

for i = 1:3
    t = t_all{i};
    y = y_all{i};

    % Overshoot
    Overshoot_K(i)   = max(0, max(y) - Tsp);
    Overshoot_pct(i) = Overshoot_K(i) / stepSize * 100;

    % Rise time (10% to 90% of the step)
    t10 = t(find(y >= T0 + 0.1*stepSize, 1));
    t90 = t(find(y >= T0 + 0.9*stepSize, 1));
    RiseTime_min(i) = t90 - t10;

    % Settling time (stays within 2% of the step around the final value)
    band  = 0.02 * stepSize;
    idx   = find(abs(y - y(end)) > band, 1, 'last');
    if isempty(idx)
        SettlingTime_min(i) = 0;
    else
        SettlingTime_min(i) = t(min(idx + 1, numel(t)));
    end

    % Steady-state error
    SSError_K(i) = Tsp - y(end);
end

results = table(Overshoot_K, Overshoot_pct, RiseTime_min, SettlingTime_min, SSError_K, ...
    'RowNames', names);
disp(results)

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
dxdt = [cstr_model(t, x(1:2), Tc); e];
end

function dxdt = cstr_pid(t, x, Kc, tauI, tauD, tauF, Tsp, Tc0)
e    = Tsp - x(2);
dTdt = (x(2) - x(4))/tauF;
Tc   = Tc0 + Kc*(e + x(3)/tauI - tauD*dTdt);
Tc   = min(max(Tc, 250), 350);
dxdt = [cstr_model(t, x(1:2), Tc); e; dTdt];
end