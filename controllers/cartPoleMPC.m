function Fx = cartPoleMPC(states)

global CP
persistent S

%% commands -----------------------------------------------------------------
if ischar(states) || isstring(states)
    Fx = [];
    switch lower(char(states))
        case 'reset'
            if ~isempty(S), S = resetStats(S); end
        case 'stats'
            Fx = solverStats(S);
        otherwise
            error('cartPoleMPC:cmd', 'Unknown command ''%s''.', char(states));
    end
    return
end
if isempty(CP), error('cartPoleMPC:init', 'Run startUp before simulating.'); end

%% (re)build when the parameters changed ------------------------------------
stamp = {CP.plant, CP.mpc, CP.ctrl.Fmax};
if isempty(S) || ~isequal(S.stamp, stamp)
    S = buildQP(CP);
    S.stamp = stamp;
end

%% solve -----------------------------------------------------------------------
t0   = tic;
e0   = states(:) - [CP.ctrl.xRef; 0; 0; 0];
c    = S.G*e0;                                % linear term of the QP
Fmax = CP.ctrl.Fmax;

U = -S.Hinv_G*e0;                             % unconstrained optimum
nIter = 0;
if any(abs(U) > Fmax)                         % force limit active: solve the QP
    if ~isempty(S.Uprev), U = S.Uprev; end    % warm start
    [U, nIter] = boxQP(S.H, c, U, -Fmax, Fmax, CP.mpc.maxIter);
end
S.Uprev = U;
Fx = U(1);

dt = toc(t0);
S.nSolve  = S.nSolve + 1;
S.nActive = S.nActive + (nIter > 0);
S.iterSum = S.iterSum + nIter;
S.iterMax = max(S.iterMax, nIter);
S.tTotal  = S.tTotal + dt;
S.tMax    = max(S.tMax, dt);
end

%% ======================================================================
function S = buildQP(CP)
p  = CP.mpc;
N  = p.N;
dt = p.dt;
[A, B] = cartPoleLinearize(CP.plant);

% zero-order-hold discretisation (base MATLAB)
E  = expm([A, B; zeros(1, 5)]*dt);
Ad = E(1:4, 1:4);
Bd = E(1:4, 5);
Qd = p.Q*dt;
Rd = p.R*dt;

% terminal weight: discrete Riccati equation solved by iteration
Pf = Qd;
for k = 1:10000
    Kd = (Rd + Bd'*Pf*Bd) \ (Bd'*Pf*Ad);
    Pn = Qd + Ad'*Pf*(Ad - Bd*Kd);
    if max(abs(Pn(:) - Pf(:))) < 1e-10*max(1, max(abs(Pn(:)))), Pf = Pn; break; end
    Pf = Pn;
end

% condensed prediction  E = Sx*e0 + Su*U,  E = [e_1; ...; e_N]
Sx = zeros(4*N, 4);
Su = zeros(4*N, N);
Ak = eye(4);
for k = 1:N
    rows = 4*(k-1) + (1:4);
    Ak = Ad*Ak;                               % Ad^k
    Sx(rows, :) = Ak;
    for j = 1:k
        Su(rows, j) = Ad^(k-j)*Bd;
    end
end
Qbar = kron(eye(N), Qd);
Qbar(end-3:end, end-3:end) = Pf;              % terminal state weighted by Pf

H = Su'*Qbar*Su + Rd*eye(N);
H = (H + H')/2;
S = struct();
S.H      = H;
S.G      = Su'*Qbar*Sx;
S.Hinv_G = H \ S.G;
S = resetStats(S);
end

function [U, nIter] = boxQP(H, c, U, lb, ub, maxIter)
%BOXQP  min 1/2*U'*H*U + c'*U  s.t.  lb <= U <= ub   (H positive definite).
%   Primal active-set method. W = variables held at a bound.
%     - minimise over the free variables; if the step would cross a bound,
%       shorten it and add that bound to W;
%     - at the minimiser of the current W, check the multipliers of W and
%       release the bound with the wrong sign; if none, U is optimal (KKT).
%   Ends at the exact solution after finitely many steps.
n   = numel(U);
lb  = lb + zeros(n, 1);   ub = ub + zeros(n, 1);
U   = min(max(U, lb), ub);
tol = 1e-9;
W   = (U <= lb + tol) | (U >= ub - tol);
atMin = false;                       % true when U minimises over the current W
for nIter = 1:maxIter
    g = H*U + c;
    if ~atMin
        Fr = ~W;
        p  = zeros(n, 1);
        if any(Fr), p(Fr) = -H(Fr, Fr) \ g(Fr); end
        ratio = inf(n, 1);                                  % distance to the bounds
        dn = Fr & (p < 0);   ratio(dn) = (lb(dn) - U(dn))./p(dn);
        up = Fr & (p > 0);   ratio(up) = (ub(up) - U(up))./p(up);
        [alpha, j] = min(ratio);
        if alpha < 1                                        % blocked by a bound
            U = U + alpha*p;
            U(j) = min(max(U(j), lb(j)), ub(j));
            W(j) = true;
        else                                                % full step
            U = U + p;
            atMin = true;
        end
        continue
    end
    % U minimises over W: multipliers of the bounds in W
    viol = zeros(n, 1);
    atL = W & (U <= lb + tol);   viol(atL) = -g(atL);       % lower bound needs g >= 0
    atU = W & (U >= ub - tol);   viol(atU) =  g(atU);       % upper bound needs g <= 0
    [vmax, i] = max(viol);
    if vmax <= tol*(1 + max(abs(g))), return; end           % KKT satisfied: optimal
    W(i) = false;                                           % release that bound
    atMin = false;
end
end

function S = resetStats(S)
S.Uprev = [];
S.nSolve = 0;  S.nActive = 0;  S.iterSum = 0;  S.iterMax = 0;
S.tTotal = 0;  S.tMax = 0;
end

function st = solverStats(S)
if isempty(S) || S.nSolve == 0
    st = struct('nSolve', 0, 'nConstrained', 0, 'meanIter', NaN, 'maxIter', 0, ...
                'meanSolveTime_ms', NaN, 'maxSolveTime_ms', NaN);
    return
end
st = struct('nSolve', S.nSolve, 'nConstrained', S.nActive, ...
            'meanIter', S.iterSum/max(S.nActive, 1), 'maxIter', S.iterMax, ...
            'meanSolveTime_ms', 1e3*S.tTotal/S.nSolve, 'maxSolveTime_ms', 1e3*S.tMax);
end
