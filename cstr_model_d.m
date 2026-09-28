function dxdt = cstr_model_d(t, x, Tc, Tf)
% Same reactor, but feed temperature Tf is now an input

q      = 100;
V      = 100;
CAf    = 1;
rho    = 1000;
Cp     = 0.239;
dHr    = -5e4;
EoverR = 8750;
k0     = 7.2e10;
UA     = 5e4;

CA = x(1);
T  = x(2);

k = k0 * exp(-EoverR / T);

dCA = (q/V)*(CAf - CA) - k*CA;
dT  = (q/V)*(Tf - T) + ((-dHr)/(rho*Cp))*k*CA + (UA/(V*rho*Cp))*(Tc - T);

dxdt = [dCA; dT];
end