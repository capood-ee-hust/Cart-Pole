function file = findLatestRun(folder, controller, q0_deg)
%FINDLATESTRUN  Newest run file of one controller at one initial angle.
%
%   file = findLatestRun(folder, 'PID', 35)
%   Looks for  <controller>_q<q0>deg_<yyyyMMdd-HHmmss>.mat  in folder and
%   returns the full path of the most recent one, or '' if there is none.

pattern = sprintf('%s_q%gdeg_*.mat', controller, q0_deg);
list = dir(fullfile(folder, pattern));
if isempty(list)
    file = '';
    return
end
names = sort({list.name});                 % timestamp format sorts by date
file  = fullfile(folder, names{end});
end
