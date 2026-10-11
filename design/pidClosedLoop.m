function [poles, Kz, Aa, Ba] = pidClosedLoop(plant, pid)


[A, B] = cartPoleLinearize(plant);
Aa = [A, zeros(4, 1);
      1, 0, 0, 0, 0];          % d/dt int(ex) = x  (xRef constant)
Ba = [B; 0];
Kz = [pid.Kp_x, -pid.Kp_q, pid.Kd_x, -pid.Kd_q, pid.Ki_x];
poles = eig(Aa + Ba*Kz);
end
