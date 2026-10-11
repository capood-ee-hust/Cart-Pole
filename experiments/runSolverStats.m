function st = runSolverStats(run)
%RUNSOLVERSTATS  Optimiser statistics stored in a run file (MPC only, else []).
%   Fields used by the reports: meanSolveTime_ms, maxSolveTime_ms.
%   Run files made before this field existed simply return [].

st = [];
if isfield(run, 'solverStats') && ~isempty(run.solverStats)
    st = run.solverStats;
end
end
