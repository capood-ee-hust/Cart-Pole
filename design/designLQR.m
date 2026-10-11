function K = designLQR(P)

[A, B] = cartPoleLinearize(P.plant);
if P.lqr.discrete
    K = lqrd(A, B, P.lqr.Q, P.lqr.R, P.ctrl.Ts);
else
    K = lqr(A, B, P.lqr.Q, P.lqr.R);
end
end
