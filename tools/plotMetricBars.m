function fig = plotMetricBars(metrics, titleText)
%PLOTMETRICBARS  Bar charts of the main performance indices, one bar per controller.
%
%   fig = plotMetricBars(metrics)            metrics from plotResults/controlMetrics
%   fig = plotMetricBars(metrics, 'q0 = 35 deg')
%
%   Panels: settling time, peak |F|, max cart excursion |x - xRef|, and the
%   control effort  int F^2 dt.  Lower is better in every panel.
%   A controller that never settles (or lets the pole fall) shows "n/a".

if nargin < 2, titleText = ''; end
names = {metrics.name};
n = numel(metrics);
panels = {'settling time [s]',               [metrics.settle];
          'peak |F| [N]',                    [metrics.peakF];
          'max |x - x_{ref}| [m]',           [metrics.xDev];
          'control effort \int F^2dt [N^2s]', [metrics.effort]};

colors = controllerStyle(names);                % same colors as plotResults
fig = figure('Name', 'Cart-pole metrics', 'Color', 'w');
for p = 1:size(panels, 1)
    ax = subplot(2, 2, p);
    hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');
    v = panels{p, 2};
    top = max([v(isfinite(v)), eps]);
    for i = 1:n
        if isfinite(v(i))
            w = 0.3;                                    % half bar width
            patch(ax, [i-w, i+w, i+w, i-w], [0, 0, v(i), v(i)], ...
                  colors(i, :), 'EdgeColor', 'none');
            text(ax, i, v(i) + 0.02*top, sprintf('%.3g', v(i)), 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', 'bottom');
        else
            text(ax, i, 0.05*top, 'n/a', 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', 'bottom');
        end
    end
    set(ax, 'XTick', 1:n, 'XTickLabel', names, 'XLim', [0.4, n + 0.6], ...
            'YLim', [0, 1.2*top]);
    title(ax, panels{p, 1});
end
if ~isempty(titleText)
    set(fig, 'Name', ['Cart-pole metrics, ' titleText]);
end
end
