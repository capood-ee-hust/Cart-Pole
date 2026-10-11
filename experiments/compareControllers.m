
if exist('collectRun', 'file') ~= 2, startUp; end      % path not set yet
cfg    = cartPoleConfig();
folder = resultsFolder(cfg);
q0     = cfg.exp.compareAngle;
if isempty(q0), q0 = cfg.sim.q0_deg; end

%% 1. pick the run files ------------------------------------------------------------
files = cfg.exp.compareFiles;
if isempty(files)
    for ctrl = {'LQR', 'PID', 'MPC'}
        f = findLatestRun(folder, ctrl{1}, q0);
        if isempty(f)
            fprintf('compareControllers: no %s run at q0 = %g deg in %s\n', ctrl{1}, q0, folder);
        else
            files{end+1} = f;                                             %#ok<SAGROW>
        end
    end
else
    for i = 1:numel(files)                       % names relative to the results folder
        if ~exist(files{i}, 'file'), files{i} = fullfile(folder, files{i}); end
    end
end
if isempty(files)
    error('compareControllers:noRuns', ...
          'No run files to compare at q0 = %g deg. Run collectData first.', q0);
end

%% 2. load ------------------------------------------------------------------------------
runs = cell(1, numel(files));
fprintf('compareControllers: q0 = %g deg\n', q0);
for i = 1:numel(files)
    S = load(files{i}, 'run');
    runs{i} = S.run;
    fprintf('  %-5s %s  (%s)\n', runs{i}.controller, files{i}, runs{i}.created);
end
checkComparable(runs);

%% 3. plot ------------------------------------------------------------------------------
c1 = runs{1}.config;
titleText = sprintf('q_0 = %g deg, T_s = %g s, F_{max} = %g N', q0, c1.ctrl.Ts, c1.ctrl.Fmax);
[metrics, figResponse] = plotResults(runs{:}, 'XRef', c1.ctrl.xRef, 'Fmax', c1.ctrl.Fmax, ...
    'SettleTol', [cfg.exp.settleTolX, cfg.exp.settleTolQ], 'Title', titleText);
figMetrics = plotMetricBars(metrics, sprintf('q0 = %g deg', q0));

fprintf('%-6s %20s   %s\n', 'run', 'simulation time [s]', 'optimiser solve time mean / max [ms]');
for i = 1:numel(runs)
    r  = runs{i};
    st = runSolverStats(r);
    solveText = '-';
    if ~isempty(st)
        solveText = sprintf('%.2f / %.2f', st.meanSolveTime_ms, st.maxSolveTime_ms);
    end
    fprintf('%-6s %20.1f   %s\n', r.controller, r.wallTime, solveText);
end

%% 4. export ------------------------------------------------------------------------------
if cfg.exp.exportFigures
    tag = fullfile(folder, sprintf('compare_q%gdeg_%s', q0, ...
                   char(datetime('now', 'Format', 'yyyyMMdd-HHmmss'))));
    saveFigure(figResponse, [tag '_response.png']);
    saveFigure(figMetrics,  [tag '_metrics.png']);
    writeMetricsCSV([tag '_metrics.csv'], metrics, runs);
    fprintf('\nSaved %s_response.png, _metrics.png, _metrics.csv\n', tag);
end
clear ctrl f i S r st c1 solveText tag titleText
