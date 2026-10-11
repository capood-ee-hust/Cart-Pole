clear;  clear global;
global CP

%% Path
projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot);
for sub = {'model', 'design', 'controllers', 'tools', 'experiments', 'STL'}
    folder = fullfile(projectRoot, sub{1});
    if exist(folder, 'dir'), addpath(folder); end    % STL: geometry of the File Solid blocks
end

%% Parameters and gains (read by the controller functions through CP)
CP = cartPoleConfig();
CP.lqr.K = designLQR(CP);
CP.pid   = designPID(CP);

%% Variables read by Cart_Pole.slx
M = CP.plant.M;   m = CP.plant.m;   l = CP.plant.l;   g = CP.plant.g;
wheel_damping = CP.plant.wheelDamping;
joint_damping = CP.plant.jointDamping;
x_0 = CP.sim.x0;
y_0 = CP.sim.y0;
q_0 = CP.sim.q0_deg;          % degrees
Ts_ctrl = CP.ctrl.Ts;         % ZOH sample time

%% Clear the PID integrator and the MPC warm start
resetControllers();
clear sub folder projectRoot
