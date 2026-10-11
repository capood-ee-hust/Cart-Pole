function [colors, lineStyles] = controllerStyle(names)
%CONTROLLERSTYLE  Color and line style of every curve in a comparison plot.
%
%   [colors, lineStyles] = controllerStyle({'LQR', 'PID', 'MPC'})
%     colors      n x 3 RGB
%     lineStyles  1 x n cell
%
%   A name starting with LQR, PID or MPC always gets the same style,
%   so a controller looks identical in every figure:
%     LQR   blue    solid        PID   orange  dashed      MPC   green  dash-dot
%   Other names, and a second curve of the same controller (e.g. two PID
%   tunings), take the next free style, so no two curves look alike.

palette = [0.165 0.471 0.839;    % LQR   #2a78d6
           0.922 0.408 0.204;    % PID   #eb6834
           0.106 0.686 0.478;    % MPC   #1baf7a
           0.290 0.227 0.655;    %       #4a3aa7
           0.929 0.631 0.000;    %       #eda100
           0.910 0.482 0.643;    %       #e87ba4
           0.890 0.286 0.282];   %       #e34948
styles  = {'-', '--', '-.', ':', '-', '--', '-.'};
known   = {'LQR', 'PID', 'MPC'};

names = cellstr(names);
n = numel(names);
slot = zeros(1, n);
% pass 1: reserved style of each known controller (first curve only)
for i = 1:n
    for k = 1:numel(known)
        if strncmpi(names{i}, known{k}, numel(known{k})) && ~any(slot == k)
            slot(i) = k;
            break
        end
    end
end
% pass 2: everything else takes the next free style
for i = find(slot == 0)
    free = setdiff([4:size(palette, 1), 1:3], slot, 'stable');   % non-reserved first
    if isempty(free)
        slot(i) = mod(i - 1, size(palette, 1)) + 1;
    else
        slot(i) = free(1);
    end
end
colors     = palette(slot, :);
lineStyles = styles(slot);
end
