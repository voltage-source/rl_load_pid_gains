%% Extract  characteristic equation coefficients
% Initial setps: define symbolic vars
    syms L_s R s zeta wn Kp Ki Kd

% Define plant model and controller
    P = 1/(s*L_s + R)
    K = Kp + Ki/s +s*Kd

% Loop transfer function and complementary sensitivity
    L = K*P
    T.eqn = L/(1+L)
    T.eqn = simplifyFraction(T.eqn)
    [T.num, T.den] = numden(T.eqn)

% Extract coefficients from numerator denominator
    T.coeff.den = coeffs(T.den,s)

% Divide coefficient vector by highest order for std form
    T.coeff.index = length(T.coeff.den)                     % Max index                     
    T.coeff.den = T.coeff.den / T.coeff.den(T.coeff.index)  % std form

%% Derive gains
% Standard form of 2nd order system
    T.std = s^2 + 2*zeta*wn*s + wn^2
    T.coeff.std = coeffs(T.std,s)

% Solve for gains
    k.diff.one = solve(T.coeff.std(1) == T.coeff.den(1),Kd) 
    k.diff.two = solve(T.coeff.std(2) == T.coeff.den(2),Kd) 


