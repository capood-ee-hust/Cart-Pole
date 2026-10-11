function [run, file] = collectRun(controller, q0_deg)
%COLLECTRUN  Simulate Cart_Pole.slx once with one controller and save the data.
%
%   [run, file] = collectRun('PID', 35)   run with PIDControl(u), q0 = 35 deg
%   [run, file] = collectRun('', 35)      run with the controller currently
%                                         selected in the Simulink block
%
%   controller  'LQR' | 'PID' | 'MPC' | ''  (empty = keep the block as it is)
%   q0_deg      initial pole angle [deg]
%
%   The simulation is configured with Simulink.SimulationInput: the
%   initial angle and, when a controller is given, the MATLAB function of
%   the controller block are changed FOR THIS RUN ONLY. The model file is
%   not modified and stays as you left it.
%
%   Each run is saved to its own file
%       <project>/<CP.exp.dir>/<controller>_q<q0>deg_<yyyyMMdd-HHmmss>.mat
%   containing the struct 'run':
%       controller, blockFcn   controller name and the call used by the block
%       q0_deg, x0             initial condition
%       t, X, F                time [s], states [x q dx dq] (N x 4), force [N]
%       fell                   true if |q| exceeded 90 deg
%       metrics                controlMetrics (settling time, peak force, ...)
%       solverStats            optimiser statistics (MPC only, else [])
%       wallTime               simulation wall-clock time [s]
%       config                 copy of CP (all parameters and gains used)
%       created                date and time of the run
%   With CP.exp.saveCSV = true a .csv with columns t,x,q,dx,dq,F is written too.

global CP
if isempty(CP), error('collectRun:init', 'Run startUp first.'); end

model = CP.sim.model;
blk   = CP.sim.ctrlBlock;
load_system(model);
if strcmpi(get_param(model, 'FastRestart'), 'on')
    error('collectRun:fastRestart', ...
        'Fast Restart is ON in %s: turn it off (Simulation tab) and run again.', model);
end

%% controller -> block call ----------------------------------------------------
in = Simulink.SimulationInput(model);
if isempty(controller)
    blockFcn   = get_param(blk, 'MATLABFcn');
    controller = controllerFromCall(blockFcn);
else
    controller = upper(char(controller));
    blockFcn   = controllerCall(controller);
    in = in.setBlockParameter(blk, 'MATLABFcn', blockFcn);   % this run only
end
in = in.setVariable('q_0', q0_deg);                           % this run only

%% simulate ----------------------------------------------------------------------
fprintf('Run %-4s  q0 = %g deg  (%s) ... ', controller, q0_deg, blockFcn);
resetControllers();
tWall = tic;
try
    out = sim(in);
catch simErr
    fprintf('FAILED\n');
    rethrow(simErr);                      % collectData prints the full cause
end
wallTime = toc(tWall);
errMsg = '';
try
    errMsg = out.ErrorMessage;
catch
end
if ~isempty(errMsg)
    fprintf('FAILED\n');
    error('collectRun:sim', 'Simulation with %s failed:\n%s', controller, errMsg);
end

%% pack the data -------------------------------------------------------------------
res = simOutToResult(out, controller);
run = struct();
run.controller = controller;
run.name       = controller;          % legend label used by plotResults
run.blockFcn   = blockFcn;
run.q0_deg     = q0_deg;
run.x0         = CP.sim.x0;
run.t          = res.t;
run.X          = res.X;
run.F          = res.F;
run.fell       = res.fell;
run.metrics    = controlMetrics(res, CP.ctrl.xRef, CP.exp.settleTolX, CP.exp.settleTolQ);
run.solverStats = [];
if strcmp(controller, 'MPC')
    run.solverStats = cartPoleMPC('stats');
end
run.wallTime   = wallTime;
run.config     = CP;
run.created    = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));

%% save, one file per run ------------------------------------------------------------
folder = resultsFolder(CP);
if ~exist(folder, 'dir'), mkdir(folder); end
stamp = char(datetime('now', 'Format', 'yyyyMMdd-HHmmss'));
base  = sprintf('%s_q%gdeg_%s', controller, q0_deg, stamp);
file  = fullfile(folder, [base '.mat']);
save(file, 'run');
if CP.exp.saveCSV
    writeRunCSV(fullfile(folder, [base '.csv']), run);
end

m = run.metrics;
fprintf('done in %.1f s | settle %s s, peak|F| %.1f N%s\n   -> %s\n', wallTime, ...
        num2str(m.settle, '%.2f'), m.peakF, ternary(run.fell, ', POLE FELL', ''), file);
end

%% =====================================================================
function call = controllerCall(name)
switch name
    case 'LQR',  call = 'LQRControl(u)';
    case 'PID',  call = 'PIDControl(u)';
    case 'MPC',  call = 'cartPoleMPC(u)';
    otherwise
        error('collectRun:controller', 'Controller must be ''LQR'', ''PID'' or ''MPC''.');
end
end

function name = controllerFromCall(call)
if     ~isempty(strfind(call, 'LQRControl')),   name = 'LQR';
elseif ~isempty(strfind(call, 'PIDControl')),   name = 'PID';
elseif ~isempty(strfind(call, 'cartPoleMPC')),  name = 'MPC';
else
    error('collectRun:block', 'The controller block calls ''%s'', not a known controller.', call);
end
end

function writeRunCSV(file, run)
fid = fopen(file, 'w');
if fid < 0, warning('collectRun:csv', 'Cannot write %s', file); return; end
fprintf(fid, 't,x,q,dx,dq,F\n');
fprintf(fid, '%.6g,%.6g,%.6g,%.6g,%.6g,%.6g\n', [run.t, run.X, run.F]');
fclose(fid);
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
