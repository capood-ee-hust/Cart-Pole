function [A, B] = cartPoleLinearize(plant)
%CARTPOLELINEARIZE  Linear model of the cart-pole about the upright equilibrium.
%
%   [A, B] = cartPoleLinearize(plant)   gives  dX/dt = A*X + B*F
%     X      [x; q; dx; dq]   (q > 0: pole leans toward -x)
%     F      force on the cart [N] (F > 0 pushes toward +x)
%     plant  struct with fields M, m, l, g, kJ   (see cartPoleConfig)
%
%   Nonlinear equations of motion (Lagrange), J = pole inertia about the
%   pivot = Jc + m*l^2 = (1 + kJ)*m*l^2:
%     [ M+m          -m*l*cos(q) ] [ddx]   [ F - m*l*sin(q)*dq^2 ]
%     [ -m*l*cos(q)   J          ] [ddq] = [ m*g*l*sin(q)        ]
%
%   About X = 0, F = 0  (sin(q) ~ q, cos(q) ~ 1, dq^2 ~ 0):
%     D0  = (M+m)*J - (m*l)^2
%     ddx = ( m^2*g*l^2 * q + J  *F ) / D0
%     ddq = ( (M+m)*m*g*l*q + m*l*F ) / D0
%   With kJ = 0 (point mass) this reduces to the textbook model
%     ddx = (m*g*q + F)/M,   ddq = ((M+m)*g*q + F)/(M*l).

M = plant.M;  m = plant.m;  l = plant.l;  g = plant.g;
J  = (1 + plant.kJ)*m*l^2;
D0 = (M + m)*J - (m*l)^2;

A = [0, 0,                   1, 0;
     0, 0,                   0, 1;
     0, m^2*g*l^2/D0,        0, 0;
     0, (M + m)*m*g*l/D0,    0, 0];
B = [0; 0; J/D0; m*l/D0];
end
