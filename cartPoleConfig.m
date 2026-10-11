function P = cartPoleConfig()

%% 1. Plant 
P.plant.M  = 0.5;    % cart mass                                    [kg]
P.plant.m  = 0.5;    % pole mass                                    [kg]
P.plant.l  = 0.3;    % distance pivot -> pole centre of mass        [m]
P.plant.g  = 9.81;   % gravity                                      [m/s^2]
P.plant.kJ = 1/12;   % pole inertia about its centre of mass: Jc = kJ*m*l^2
                     %   0     point mass on a massless rod
                     %   1/3   uniform rod of length 2*l, pivot at one end
                     %   1/12  original derivation of the repo (J_pivot = 13/12*m*l^2)
                     % -> check the pole geometry in Cart_Pole.slx
P.plant.wheelDamping = 1e-5;   % Simscape wheel joint damping
P.plant.jointDamping = 1e-5;   % Simscape pole joint damping

%% 2. Simulink model and initial condition 
P.sim.model     = 'Cart_Pole';                                     % .slx name
P.sim.ctrlBlock = 'Cart_Pole/controllers/Interpreted MATLAB Function';
P.sim.x0     = 0;      % initial cart position                [m]    -> x_0
P.sim.y0     = 0.125;  % cart height used by Simscape         [m]    -> y_0
P.sim.q0_deg = 35;     % initial pole angle                   [deg]  -> q_0

%% 3. Settings shared by all controllers ----------------------------------
P.ctrl.Ts   = 0.01;    % controller sample time [s]; ZOH block uses Ts_ctrl
P.ctrl.Fmax = 30;      % actuator force limit for EVERY controller [N] (Inf = none)
P.ctrl.xRef = 0;       % cart position set-point [m]

%% 4. LQR:  F = -K*(X - X_ref),  K minimises  int( e'*Q*e + R*F^2 ) dt 
P.lqr.Q        = diag([100, 100, 10, 10]);  % weights on [x q dx dq]
P.lqr.R        = 1;                         % weight on F
P.lqr.discrete = false;   % true: design for the sampled loop (lqrd with Ts)

%% 5. PID (two loops, all gains positive) 
%   F = -( Kp_q*q + Kd_q*dq )                       angle loop    (PD)
%       +( Kp_x*ex + Ki_x*int(ex) + Kd_x*dx )       position loop (PID)
%   ex = x - xRef.  D terms use the measured velocities dx, dq.
P.pid.method = 'manual';  % 'manual' : use the five gains below
                          % 'lqi'    : compute them from the LQI weights below
P.pid.Kp_q = 42;    % [N/rad]
P.pid.Kd_q = 7.5;   % [N s/rad]
P.pid.Kp_x = 9.2;   % [N/m]
P.pid.Ki_x = 5.2;   % [N/(m s)]
P.pid.Kd_x = 7.5;   % [N s/m]
P.pid.Kaw  = 1;     % anti-windup back-calculation gain [1/s] (0 = off)

% used only when method = 'lqi'; weights on [x q dx dq int(ex)]
P.pid.lqi.Q = diag([100, 100, 10, 10, 100]);
P.pid.lqi.R = 1;

% mass grid used by tunePIDGains (optional tool, not run by startUp)
P.pid.robust.cartMassRatio = [0.5, 1, 2, 3];   % cart mass x nominal
P.pid.robust.poleMassRatio = [0.5, 1, 2];      % pole mass x nominal

%% 6. Linear MPC (cartPoleMPC, plain MATLAB) 
%   Prediction with the linearised model, discretised with step dt:
%   min  sum_k dt*( e_k'*Q*e_k + R*F_k^2 ) + e_N'*Pf*e_N ,  |F_k| <= Fmax
%   e = X - X_ref,  Pf = discrete LQR cost-to-go (Riccati).
P.mpc.N       = 25;                       % horizon steps
P.mpc.dt      = 0.04;                     % prediction step [s]  (horizon = N*dt)
P.mpc.Q       = diag([600, 1000, 100, 40]);
P.mpc.R       = 1e-3;
P.mpc.maxIter = 100;                      % active-set iterations per sample (safety cap)

%% 7. Experiments: collectData / compareControllers
P.exp.dir          = 'results';  % run files and figures (relative to the project)
P.exp.controllers  = {};         % collectData:  {}  -> the controller selected in
                                 %   the Simulink block, one run
                                 %   e.g. {'LQR','PID','MPC'} -> one after another
                                 %   (block switched for each run only, .slx not modified)
P.exp.angles       = [];         % collectData: initial angles q0 [deg], one run per
                                 %   angle and controller; [] -> sim.q0_deg
P.exp.compareAngle = [];         % compareControllers: q0 [deg] of the runs to
                                 %   compare; [] -> sim.q0_deg
P.exp.compareFiles = {};         % compareControllers: explicit run files; {} ->
                                 %   newest file of each controller at compareAngle
P.exp.saveCSV      = false;      % collectData: also write t,x,q,dx,dq,F as .csv
P.exp.exportFigures = true;      % compareControllers: save figures (.png) and table (.csv)
P.exp.settleTolX   = 0.02;       % settling band on the cart position [m]
P.exp.settleTolQ   = 1;          % settling band on the pole angle [deg]
end
