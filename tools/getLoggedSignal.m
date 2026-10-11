function [t, data] = getLoggedSignal(out, name, Ts)
%GETLOGGEDSIGNAL  Read a 'To Workspace' signal whatever its save format.
%
%   [t, data] = getLoggedSignal(out, 'state_data', Ts)
%     out   Simulink.SimulationOutput (or a struct with the same fields)
%     name  variable name of the To Workspace block
%     Ts    sample period, used to rebuild time when it was not logged
%   data has one row per sample (N x width), t is N x 1.
%   Handles the formats Array, Timeseries and Structure (With Time).

if isstruct(out)
    v = out.(name);
else
    v = out.get(name);
end

if isa(v, 'timeseries')
    t = v.Time;
    data = squeeze(v.Data);
    if size(data, 1) ~= numel(t), data = data.'; end
elseif isstruct(v) && isfield(v, 'signals')
    data = v.signals.values;
    t = [];
    if isfield(v, 'time'), t = v.time; end
else
    data = v;
    t = [];
end
if isvector(data), data = data(:); end

N = size(data, 1);
if isempty(t) || numel(t) ~= N
    t = [];
    try                                     % array format: time may be in tout
        tout = out.get('tout');
        if numel(tout) == N, t = tout; end
    catch
    end
    if isempty(t), t = (0:N - 1)'*Ts; end
end
t = t(:);
end
