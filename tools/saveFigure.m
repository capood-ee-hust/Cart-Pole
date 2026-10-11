function saveFigure(fig, file, dpi)
%SAVEFIGURE  Save a figure as an image (PNG by default, format from the extension).
%   saveFigure(fig, 'results/compare.png')       200 dpi
%   saveFigure(fig, 'results/compare.pdf', 300)

if nargin < 3, dpi = 200; end
if exist('exportgraphics', 'file')                 % MATLAB R2020a and later
    exportgraphics(fig, file, 'Resolution', dpi);
else
    [~, ~, ext] = fileparts(file);
    print(fig, ['-d' lower(ext(2:end))], sprintf('-r%d', dpi), file);
end
end
