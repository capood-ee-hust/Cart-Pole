function res = simOutToResult(out, name)
%SIMOUTTORESULT  Convert a Simulink output to a plain result struct.
%
%   res = simOutToResult(out, 'PID (Simscape)')
%   Reads out.state_data ([x q dx dq]) and out.control_input_data (F),
%   the To Workspace variables of Cart_Pole.slx, and returns
%   res.t, res.X (N x 4), res.F (N x 1), res.name, res.fell.

global CP
Ts = 0.01;
if ~isempty(CP), Ts = CP.ctrl.Ts; end
if nargin < 2, name = 'Simulink'; end

[t, X] = getLoggedSignal(out, 'state_data', Ts);
[~, F] = getLoggedSignal(out, 'control_input_data', Ts);
n = min(size(X, 1), numel(F));
res = struct('t', t(1:n), 'X', X(1:n, :), 'F', F(1:n), 'name', name, ...
             'fell', any(abs(X(1:n, 2)) > pi/2));
end
