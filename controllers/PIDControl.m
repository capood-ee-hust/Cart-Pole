function Fx = PIDControl(states)

global CP
persistent I
if isempty(I), I = 0; end


if ischar(states) || isstring(states)
    switch lower(char(states))
        case 'reset',  I = 0;  Fx = [];
        case 'state',  Fx = I;
        otherwise,     error('PIDControl:cmd', 'Unknown command ''%s''.', char(states));
    end
    return
end
if isempty(CP), error('PIDControl:init', 'Run startUp before simulating.'); end


p    = CP.pid;
Ts   = CP.ctrl.Ts;
Fmax = CP.ctrl.Fmax;

x = states(1);  q = states(2);  dx = states(3);  dq = states(4);
ex = x - CP.ctrl.xRef;

F_angle = -(p.Kp_q*q + p.Kd_q*dq);
F_pos   =   p.Kp_x*ex + I + p.Kd_x*dx;
F_unsat = F_angle + F_pos;
Fx      = min(max(F_unsat, -Fmax), Fmax);


I = I + Ts*( p.Ki_x*ex + p.Kaw*(Fx - F_unsat) );
end
