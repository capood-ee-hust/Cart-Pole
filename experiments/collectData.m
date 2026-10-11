%COLLECTDATA  Run the Simulink model and save every run to its own file.
%
%   Settings: cartPoleConfig, section 7 (P.exp)
%     P.exp.controllers = {}                    one run with the controller
%                                               selected in the Simulink block
%     P.exp.controllers = {'LQR','PID','MPC'}   these controllers, one after
%                                               another (the block is switched for
%                                               each run only; the .slx is not modified)
%     P.exp.angles      = []                    q0 from P.sim.q0_deg, or a list,
%                                               e.g. [15 35 50] for one run per angle
%
%   Output: one file per run in <project>/results, named
%       <controller>_q<q0>deg_<yyyyMMdd-HHmmss>.mat     (struct 'run', see collectRun)
%   Then run compareControllers to plot the comparison.
%
%   Turn Fast Restart OFF in Simulink before running this script.

startUp;                                     % fresh parameters and designs

controllerList = CP.exp.controllers;
if isempty(controllerList), controllerList = {''}; end      % '' = as set in the block
angleList = CP.exp.angles;
if isempty(angleList), angleList = CP.sim.q0_deg; end

fprintf('collectData: %d run(s) -> %s\n', numel(controllerList)*numel(angleList), ...
        resultsFolder(CP));
savedFiles = {};
failedRuns = {};
for iAngle = 1:numel(angleList)
    for iCtrl = 1:numel(controllerList)
        try
            [~, runFile] = collectRun(controllerList{iCtrl}, angleList(iAngle));
            savedFiles{end+1} = runFile;                                   %#ok<SAGROW>
        catch runErr                     % keep going with the other runs
            runName = char(controllerList{iCtrl});
            if isempty(runName), runName = 'block controller'; end
            failedRuns{end+1} = sprintf('%s at q0 = %g deg: %s', ...       %#ok<SAGROW>
                runName, angleList(iAngle), runErr.message);
            fprintf(2, 'Run failed - %s\n%s', runName, errorReport(runErr));
            lastRunError = runErr;       % kept in the workspace for inspection
        end
    end
end
fprintf('collectData: %d file(s) saved, %d run(s) failed. Next: compareControllers\n', ...
        numel(savedFiles), numel(failedRuns));
clear iAngle iCtrl controllerList angleList runFile runErr runName
