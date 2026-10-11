function opt = parseOptions(opt, varargin)
%PARSEOPTIONS  Overwrite default options with name-value pairs.
%
%   opt = parseOptions(defaults, 'Name1', value1, 'Name2', value2, ...)
%   Names are matched against the fields of defaults, exactly first, then
%   case-insensitively; an unknown or ambiguous name is an error.

if mod(numel(varargin), 2) ~= 0
    error('parseOptions:pairs', 'Options must be given as name-value pairs.');
end
names = fieldnames(opt);
for k = 1:2:numel(varargin)
    name = char(varargin{k});
    hit  = find(strcmp(names, name));
    if isempty(hit), hit = find(strcmpi(names, name)); end
    if numel(hit) > 1
        error('parseOptions:ambiguous', 'Option ''%s'' is ambiguous.', name);
    end
    if isempty(hit)
        error('parseOptions:unknown', 'Unknown option ''%s''. Valid: %s', ...
              char(varargin{k}), strjoin(names', ', '));
    end
    opt.(names{hit}) = varargin{k+1};
end
end
