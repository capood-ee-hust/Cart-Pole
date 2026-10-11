function folder = resultsFolder(cfg)
%RESULTSFOLDER  Absolute path of the folder holding the run files.
%   folder = resultsFolder(CP)   ->  <project folder>/<CP.exp.dir>

root   = fileparts(which('cartPoleConfig'));
folder = fullfile(root, cfg.exp.dir);
end
