function [metrics, fig] = plotResults(varargin)
%PLOTRESULTS  Plot and compare closed-loop responses, print performance indices.
%
%   plotResults(out)                      Simulink output of Cart_Pole.slx
%   plotResults(out1, out2, ...)          several runs overlaid
%   plotResults(run1, run2, ...)          run structs saved by collectData
%   [m, fig] = plotResults(...)           struct array of controlMetrics, figure
%
%   Options, given after the data:
%     'Names'      legend labels, one per input      (default: names in the data)
%     'XRef'       cart set-point [m]                (default: CP.ctrl.xRef, else 0)
%     'Fmax'       force limit drawn on F [N]        (default: CP.ctrl.Fmax, else Inf)
%     'SettleTol'  settling band [tolX_m, tolQ_deg]  (default: CP.exp settings, else [0.02 1])
%     'Title'      text above the first plot         (default: none)
%
%   Example (compare two Simulink runs):
%     % run with LQRControl(u), then:   outLQR = out;
%     % switch the block to PIDControl(u), run again, then:
%     plotResults(outLQR, out, 'Names', {'LQR', 'PID'})
%   For a saved, three-controller comparison use collectData + compareControllers.

global CP
opt = struct('Names', {{}}, 'XRef', 0, 'Fmax', Inf, 'SettleTol', [0.02 1], 'Title', '');
if ~isempty(CP)
    opt.XRef = CP.ctrl.xRef;
    opt.Fmax = CP.ctrl.Fmax;
    opt.SettleTol = [CP.exp.settleTolX, CP.exp.settleTolQ];
end

%% inputs: data first, then name-value options ------------------------------------
firstOpt = find(cellfun(@(a) ischar(a) || isstring(a), varargin), 1);
if isempty(firstOpt), firstOpt = numel(varargin) + 1; end
data = varargin(1:firstOpt - 1);
opt  = parseOptions(opt, varargin{firstOpt:end});
names = cellstr(opt.Names);

n = numel(data);
if n == 0, error('plotResults:noData', 'Nothing to plot.'); end
res = cell(1, n);
for i = 1:n
    r = data{i};
    if ~isstruct(r) || ~isfield(r, 't')           % Simulink output
        r = simOutToResult(r, sprintf('Simulink %d', i));
    end
    if i <= numel(names), r.name = names{i}; end
    res{i} = r;
end

%% figure -------------------------------------------------------------------------
fig = figure('Name', 'Cart-pole response', 'Color', 'w');
labels = {'cart position x [m]', 'pole angle q [deg]', 'force F [N]'};
ax = [];
for s = 1:3
    ax(s) = subplot(3, 1, s);
    hold(ax(s), 'on');  grid(ax(s), 'on');  box(ax(s), 'on');
    ylabel(ax(s), labels{s});
end
[colors, lineStyles] = controllerStyle(cellfun(@(r) r.name, res, 'UniformOutput', false));
for i = 1:n
    r  = res{i};
    c  = colors(i, :);
    ls = lineStyles{i};
    plot(ax(1), r.t, r.X(:, 1),          'Color', c, 'LineStyle', ls, 'LineWidth', 1.6, ...
         'DisplayName', r.name);
    plot(ax(2), r.t, rad2deg(r.X(:, 2)), 'Color', c, 'LineStyle', ls, 'LineWidth', 1.6);
    stairs(ax(3), r.t, r.F,              'Color', c, 'LineStyle', ls, 'LineWidth', 1.3);
end
grey = [0.45 0.45 0.45];
plot(ax(1), xlim(ax(1)), [opt.XRef opt.XRef], ':', 'Color', grey, 'HandleVisibility', 'off');
plot(ax(2), xlim(ax(2)), [0 0], ':', 'Color', grey);
if isfinite(opt.Fmax)
    plot(ax(3), xlim(ax(3)),  [opt.Fmax opt.Fmax], '--', 'Color', grey);
    plot(ax(3), xlim(ax(3)), -[opt.Fmax opt.Fmax], '--', 'Color', grey);
end
xlabel(ax(3), 'time [s]');
legend(ax(1), 'Location', 'best');
if ~isempty(opt.Title), title(ax(1), opt.Title); end
linkaxes(ax, 'x');

%% metrics --------------------------------------------------------------------------
metrics = cellfun(@(r) controlMetrics(r, opt.XRef, opt.SettleTol(1), opt.SettleTol(2)), ...
                  res, 'UniformOutput', false);
metrics = [metrics{:}];
fprintf('\n%-20s %10s %18s %18s %10s %12s\n', 'response', 'peak|F|', 'x range', ...
        'q min / max', 'settle', 'int F^2 dt');
fprintf('%-20s %10s %18s %18s %10s %12s\n', '', '[N]', '[m]', '[deg]', '[s]', '[N^2 s]');
for i = 1:n
    m = metrics(i);
    fprintf('%-20s %10.1f %8.3f .. %6.3f %8.1f / %6.1f %10s %12.1f%s\n', m.name, m.peakF, ...
            m.xMin, m.xMax, m.qMin, m.qMax, num2str(m.settle, '%.2f'), m.effort, ...
            ternary(m.fell, '   FELL', ''));
end
fprintf('settle: |x - xRef| <= %g m and |q| <= %g deg until the end of the run\n\n', ...
        opt.SettleTol(1), opt.SettleTol(2));
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
