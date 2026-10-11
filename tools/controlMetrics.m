function m = controlMetrics(res, xRef, tolX, tolQdeg)
%CONTROLMETRICS  Performance indices of one closed-loop response.
%
%   m = controlMetrics(res)                        res from simOutToResult
%   m = controlMetrics(res, xRef, tolX, tolQdeg)   (defaults 0, 0.02 m, 1 deg)
%
%   m.peakF        max |F|                                       [N]
%   m.xMin, m.xMax cart excursion                                [m]
%   m.xDev         max |x - xRef|                                [m]
%   m.qMin, m.qMax pole angle extremes                           [deg]
%   m.settle       first time after which |x - xRef| <= tolX AND
%                  |q| <= tolQdeg for the rest of the run        [s]
%                  (NaN: never settles within the run)
%   m.effort       integral of F^2 dt                            [N^2 s]
%   m.xFinal       final cart position                           [m]
%   m.fell         true if |q| exceeded 90 deg

if nargin < 2 || isempty(xRef),    xRef = 0;     end
if nargin < 3 || isempty(tolX),    tolX = 0.02;  end
if nargin < 4 || isempty(tolQdeg), tolQdeg = 1;  end

t = res.t(:);  X = res.X;  F = res.F(:);
okF = ~isnan(F);

m.name  = res.name;
m.fell  = any(abs(X(:, 2)) > pi/2);
m.peakF = max(abs(F(okF)));
m.xMin  = min(X(:, 1));       m.xMax = max(X(:, 1));
m.xDev  = max(abs(X(:, 1) - xRef));
m.qMin  = rad2deg(min(X(:, 2)));  m.qMax = rad2deg(max(X(:, 2)));
m.effort = trapz(t(okF), F(okF).^2);
m.xFinal = X(end, 1);

outside = abs(X(:, 1) - xRef) > tolX | abs(rad2deg(X(:, 2))) > tolQdeg;
last = find(outside, 1, 'last');
if isempty(last)
    m.settle = 0;
elseif last == numel(t) || m.fell
    m.settle = NaN;
else
    m.settle = t(last + 1);
end
end
