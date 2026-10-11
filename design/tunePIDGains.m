function pid = tunePIDGains(P, varargin)

opt = struct('Iterations', 150000, 'Seed', 11, 'ZetaMin', 0.5, 'KpqMax', 45, ...
             'Margin', 0.05, 'CartMassRatio', P.pid.robust.cartMassRatio, ...
             'PoleMassRatio', P.pid.robust.poleMassRatio);
opt = parseOptions(opt, varargin{:});

% search box: [Kp_q Kd_q Kp_x Ki_x Kd_x] = lo + span.*rand
lo   = [10,  0.5,  0.5, 0.1,  0.5];
span = [70, 10.0, 20.0, 6.0, 20.0];

% robustness grid of plants, built once
plants = {};
for rM = opt.CartMassRatio
    for rm = opt.PoleMassRatio
        pl = P.plant;  pl.M = rM*pl.M;  pl.m = rm*pl.m;
        plants{end+1} = pl;                                                
    end
end

rng(opt.Seed);
best = [];  bestCost = Inf;  nKept = 0;
for k = 1:opt.Iterations
    G = lo + span.*rand(1, 5);
    if G(1) > opt.KpqMax, continue; end
    pid = gainsToStruct(G);
    p = pidClosedLoop(P.plant, pid);
    if min(-real(p)./abs(p)) < opt.ZetaMin, continue; end
    worst = -Inf;
    for c = 1:numel(plants)
        worst = max(worst, max(real(pidClosedLoop(plants{c}, pid))));
        if worst > -opt.Margin, break; end
    end
    if worst > -opt.Margin, continue; end
    nKept = nKept + 1;
    cost  = max(real(p)) + 0.5*worst;
    if cost < bestCost, bestCost = cost;  best = G; end
end

if isempty(best)
    error('tunePIDGains:none', 'No candidate met the constraints; relax them.');
end
pid = gainsToStruct(best);
fprintf('tunePIDGains: %d of %d candidates met the constraints. Best:\n', nKept, opt.Iterations);
fprintf('P.pid.Kp_q = %.2f;  P.pid.Kd_q = %.2f;  P.pid.Kp_x = %.2f;  P.pid.Ki_x = %.2f;  P.pid.Kd_x = %.2f;\n', best);
end

function pid = gainsToStruct(G)
pid = struct('Kp_q', G(1), 'Kd_q', G(2), 'Kp_x', G(3), 'Ki_x', G(4), 'Kd_x', G(5));
end
