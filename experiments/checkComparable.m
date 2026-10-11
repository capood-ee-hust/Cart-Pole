function ok = checkComparable(runs)
%CHECKCOMPARABLE  Warn when runs were not made under the same conditions.
%
%   ok = checkComparable(runs)      runs = cell array of run structs
%   Compares, against the first run: initial condition, plant parameters,
%   sample time, force limit and set-point. A comparison between
%   controllers is only fair when all of these are equal.

checks = {'initial angle q0',           @(r) r.q0_deg;
          'initial cart position x0',   @(r) r.x0;
          'plant parameters',           @(r) r.config.plant;
          'sample time Ts',             @(r) r.config.ctrl.Ts;
          'force limit Fmax',           @(r) r.config.ctrl.Fmax;
          'cart set-point xRef',        @(r) r.config.ctrl.xRef};
ok = true;
for c = 1:size(checks, 1)
    ref = checks{c, 2}(runs{1});
    for i = 2:numel(runs)
        if ~isequal(checks{c, 2}(runs{i}), ref)
            warning('checkComparable:differ', ...
                '%s differs between %s and %s: the comparison is not fair.', ...
                checks{c, 1}, runs{1}.controller, runs{i}.controller);
            ok = false;
        end
    end
end
names = cellfun(@(r) r.controller, runs, 'UniformOutput', false);
if numel(unique(names)) < numel(names)
    warning('checkComparable:duplicate', 'The same controller appears more than once.');
end
end
