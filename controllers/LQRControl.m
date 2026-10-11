function Fx = LQRControl(states)


global CP
if isempty(CP), error('LQRControl:init', 'Run startUp before simulating.'); end

X    = states(:);
Xref = [CP.ctrl.xRef; 0; 0; 0];
Fmax = CP.ctrl.Fmax;

Fx = -CP.lqr.K*(X - Xref);
Fx = min(max(Fx, -Fmax), Fmax);
end
