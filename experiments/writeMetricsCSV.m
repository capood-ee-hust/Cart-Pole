function writeMetricsCSV(file, metrics, runs)
%WRITEMETRICSCSV  Table of performance indices, one row per controller.
%
%   writeMetricsCSV(file, metrics, runs)
%   Columns: controller, q0_deg, settle_s, peakF_N, xDev_m, qMin_deg, qMax_deg,
%            effort_N2s, fell, wallTime_s, solveMean_ms, solveMax_ms (MPC only)

fid = fopen(file, 'w');
if fid < 0
    warning('writeMetricsCSV:open', 'Cannot write %s', file);
    return
end
fprintf(fid, ['controller,q0_deg,settle_s,peakF_N,xDev_m,qMin_deg,qMax_deg,' ...
              'effort_N2s,fell,wallTime_s,solveMean_ms,solveMax_ms\n']);
for i = 1:numel(metrics)
    m = metrics(i);  r = runs{i};
    st = runSolverStats(r);
    solveMean = NaN;  solveMax = NaN;
    if ~isempty(st)
        solveMean = st.meanSolveTime_ms;  solveMax = st.maxSolveTime_ms;
    end
    fprintf(fid, '%s,%g,%.4g,%.4g,%.4g,%.4g,%.4g,%.4g,%d,%.3g,%.3g,%.3g\n', ...
            m.name, r.q0_deg, m.settle, m.peakF, m.xDev, m.qMin, m.qMax, ...
            m.effort, m.fell, r.wallTime, solveMean, solveMax);
end
fclose(fid);
end
