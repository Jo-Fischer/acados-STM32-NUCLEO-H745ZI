% Author: Jörg Fischer
% -------------------------------------------------------------------------
% This script uses the MATLAB acados interface on windows 11 to generate
% the c cource code included in the STM32 firmware. The gerenerated c-files
% are already generated and part of the repository so that it is not needed
% to run this script to compile the c-code. It only shows how the code was
% generated
% --------------------------------------------------------------------------
% all paths in this script are relative to the folder where this script is
% located. Hence, this folder is made the work space folder
cd(fileparts(mfilename('fullpath')));

% to use this script, the acados MATLAB interface must be installed
% --> run the following ONCE after check out of acados
%addpath('../acados/interfaces/acados_matlab_octave/');
%acados_install_windows('-DBLASFEO_TARGET=GENERIC');

% set environment variables
cd(fileparts(mfilename('fullpath')));
run('../acados/interfaces/acados_matlab_octave/acados_env_variables_windows.m')

clear all; clc;
import casadi.*

% code generation destination folder is reltive to work space path 
% --> make folder of this script to work space folder
cd(fileparts(mfilename('fullpath')));

% code is generated into a folder in the firwware folder
generated_nmpc_c_code_path = '../STM32CubeIDE/CM7/Core/USER_CODE/acados_generated/mpc';

% options needed for the Simulink example
if ~exist('simulink_opts','var')
    % disp('using acados simulink default options')
    % simulink_opts = get_acados_simulink_opts;
    disp('using empty simulink_opts to generate solver without simulink block')
    simulink_opts = [];
end
check_acados_requirements();
ocp = AcadosOcp();
ocp.code_export_directory = generated_nmpc_c_code_path;


%% solver settings
T = 0.2;      % [s] prediction horizon length
N = 10;       % number of discretization steps
x0 = [0.1; 0; 0; 0]; % initial state
U_max = 10;   % inpput constraints (+U_max/-U_max)

radDevMax = 5/180 * pi;
radPerSecDevMax = (0.14*pi);
W_x = [10,                   0,             0, 0;  
     0, 5,             0, 0;
     0,                   0, 100, 0;
     0,                   0,             0, 1];
W_u = 0.01;
terminalCost = W_x;

% Set reference for states [theta1, theta1p, theta2, theta2p] and u
ocp.cost.yref_0 = 0;  % only u
ocp.cost.yref =   [pi; 0; pi; 0; 0];  % x and u
ocp.cost.yref_e = [pi; 0; pi; 0]; % only x

N_short = 0;
T_short = 0.012;
N_long = N - N_short;
T_long = (T-T_short*N_short)/N_long;
steps_short  = 4; 
steps_long   = 4; 
timeSteps_Ctrl=[ones(1,N_short).*T_short,ones(1,N_long).*T_long];
intStages_Ctrl = ones(N,1) * 4;
intSteps_Ctrl =  ones(N,1) * steps_short;
intSteps_Ctrl(N_short+1:end) = steps_long;
t_grid_ctrl = cumsum([0, timeSteps_Ctrl]);

nlp_solver_max_iter = 1000;
globalization = 'FIXED_STEP';
nlp_solver = 'SQP_RTI'; 
qp_solver = 'FULL_CONDENSING_HPIPM'; % dense QP, faster for small problems
hessianApprox='EXACT';
integratorType = 'ERK';
compFlag = '-O3';


%% model dynamics
model = get_NonLinPendulumModel_Coulomb();
nx = length(model.x); % state size
nu = length(model.u); % input size
ocp.model = model;

x=model.x;
u=model.u;

%% Initial stage cost (only on u_0)
ny_0 = nx;
ocp.cost.cost_type_0 = 'LINEAR_LS';
ocp.cost.Vx_0 = eye(nx);
ocp.cost.Vu_0 = zeros(nx,nu);
ocp.cost.W_0  = 1e-2*eye(nx);
ocp.cost.yref_0 = x0;

%% Path cost (on x and u)
ocp.cost.cost_type = 'NONLINEAR_LS';
model.cost_y_expr = [pi*(1+cos(x(1)/2)); x(2); pi*(1+cos(x(3)/2)); x(4); u];
ocp.cost.W = blkdiag(W_x, W_u);

%% Terminal cost (on x only)
ny_e = nx;
ocp.cost.cost_type_e = 'NONLINEAR_LS';
model.cost_y_expr_e = [pi*(1+cos(x(1)/2)); x(2); pi*(1+cos(x(3)/2)); x(4)];
ocp.cost.W_e = terminalCost;
ocp.cost.Vx_e = eye(nx);

%% define linear constraints
ocp.constraints.idxbu = 0;
ocp.constraints.lbu = -U_max;
ocp.constraints.ubu = U_max;
ocp.constraints.x0 = x0;

%% define solver options for embedded use
ocp.solver_options.N_horizon = N;
ocp.solver_options.tf = T;
ocp.solver_options.nlp_solver_type = nlp_solver;
ocp.solver_options.qp_solver = qp_solver;
ocp.solver_options.hessian_approx = hessianApprox;
ocp.solver_options.ext_fun_compile_flags = compFlag;
ocp.solver_options.nlp_solver_max_iter = nlp_solver_max_iter;
ocp.simulink_opts = simulink_opts;
ocp.solver_options.globalization = globalization;
ocp.solver_options.globalization_alpha_min = 1e-3;
ocp.solver_options.integrator_type = integratorType;
ocp.solver_options.sim_method_num_stages = intStages_Ctrl;
ocp.solver_options.sim_method_num_steps = intSteps_Ctrl;
if exist('timeSteps_Ctrl', 'var')
    ocp.solver_options.time_steps = timeSteps_Ctrl;
end
ocp.solver_options.regularize_method = 'MIRROR';  
ocp.solver_options.reg_epsilon = 1e-1;


%% create solver
ocp_solver = AcadosOcpSolver(ocp);


%% solver initial guess 
u_traj_init = zeros(1,N);
x_traj_init = repmat(x0, 1, N+1);

ocp_solver.set('constr_x0', x0);
ocp_solver.set('init_x', vertcat(x_traj_init));
ocp_solver.set('init_u', vertcat(u_traj_init));
ocp_solver.set('init_pi', zeros(nx,N)); % multipliers


%% solve OCP
ocp_solver.solve();
 
% get solution
utraj = ocp_solver.get('u');
xtraj = ocp_solver.get('x');

status_solver = ocp_solver.get('status'); % 0 - success
if( status_solver ~= 0 )
    sprintf('solver failed with status %d', status_solver)
    ocp_solver.set('qp_print_level', 2);
    ocp.solver_options.print_level = 2;
    error('not solved');
end

%% plot solution
if ~exist('t_grid_ctrl', 'var')
    t_grid_ctrl = 0:T/N:T;
end    
figure; hold on;
states = {'theta1', 'theta1p', 'theta2', 'theta2p'};
for i=1:length(states)
    subplot(length(states)+1, 1, i);
    plot(t_grid_ctrl, xtraj(i,:)); grid on;
    ylabel(states{i});
    xlabel('t [s]')
end
subplot(5,1,5)
u_opt_ol = ocp_solver.get('u');
stairs(t_grid_ctrl, [u_opt_ol'; 0])
ylabel('u [V]')
xlabel('t [s]')
grid on
