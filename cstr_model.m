function dxdt = cstr_model(t, x, Tc)
% x(1) = CA (mol/L), x(2) = T (K), Tc = coolant temperature (K)

% Parameters
q      = 100;      % feed flow rate (L/min)
V      = 100;      % reactor volume (L)
CAf    = 1;        % feed concentration (mol/L)
Tf     = 350;      % feed temperature (K)
rho    = 1000;     % density (g/L)
Cp     = 0.239;    % heat capacity (J/g.K)
dHr    = -5e4;     % heat of reaction (J/mol)
EoverR = 8750;     % activation energy / R (K)
k0     = 7.2e10;   % pre-exponential factor (1/min)
UA     = 5e4;      % heat transfer coefficient x area (J/min.K)

CA = x(1);
T  = x(2);

k = k0 * exp(-EoverR / T);   % Arrhenius rate constant

dCA = (q/V)*(CAf - CA) - k*CA;
dT  = (q/V)*(Tf - T) + ((-dHr)/(rho*Cp))*k*CA + (UA/(V*rho*Cp))*(Tc - T);

dxdt = [dCA; dT];
end