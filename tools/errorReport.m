function txt = errorReport(ME, indent)
%ERRORREPORT  Message of an error together with all its causes, without stack.
%
%   fprintf('%s', errorReport(ME))
%   Simulink often reports a short top-level message ("Simulation aborted")
%   and puts the real reason in ME.cause; this prints the whole chain:
%       Simulation aborted
%         Caused by: <reason 1>
%           Caused by: <reason of reason 1>

if nargin < 2, indent = ''; end
if isempty(indent)
    txt = sprintf('%s\n', ME.message);
else
    txt = sprintf('%sCaused by: %s\n', indent, ME.message);
end
for k = 1:numel(ME.cause)
    txt = [txt, errorReport(ME.cause{k}, [indent '  '])];                 %#ok<AGROW>
end
end
