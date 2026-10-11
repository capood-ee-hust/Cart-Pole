function pid = designPID(P)

pid = P.pid;
switch lower(pid.method)
    case 'manual'
        % nothing to compute
    case 'lqi'
        [~, ~, Aa, Ba] = pidClosedLoop(P.plant, pid);
        Kz = -lqr(Aa, Ba, pid.lqi.Q, pid.lqi.R);       % F = Kz*z
        pid.Kp_x =  Kz(1);   pid.Kp_q = -Kz(2);
        pid.Kd_x =  Kz(3);   pid.Kd_q = -Kz(4);
        pid.Ki_x =  Kz(5);
    otherwise
        error('designPID:method', 'P.pid.method must be ''manual'' or ''lqi''.');
end
end
