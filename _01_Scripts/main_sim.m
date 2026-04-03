% % -------------------------------------------------------------------------
% % OVAP-X1: Full Mission Simulation
% % File Name: main_sim.m
% % Position: Root > main_sim.m
% % Description: Main Loop. Fixed Coaxial Array dimension routing (8x1 thrusts).
% % -------------------------------------------------------------------------
% clear; clc; close all;
% 
% % --- 1. CONFIG PATHS ---
% addpath(genpath('config'));
% addpath(genpath('modules'));     
% addpath(genpath('utils'));       
% 
% fprintf('Initializing OVAP-X1 Simulation...\n');
% 
% % Load Parameters
% % sys = init_params();  
% % dt  = 0.002; % 500Hz
% % sys.sim.dt = dt; 
% % ctrl_params = controller_params(sys);
% % traj_limits = init_limits();
% % Load Parameters
% sys = init_params();  
% dt  = sys.sim.dt; 
% ctrl_params = controller_params(sys);
% traj_limits = init_limits();
% 
% % --- [BỔ SUNG] KIỂM TRA MISSION FEASIBILITY TRƯỚC KHI BAY ---
% is_safe = mission_feasibility_check(sys, traj_limits);
% if ~is_safe
%     % Bạn có thể đặt pause ở đây để hệ thống dừng lại đợi bạn xác nhận
%     warning('Mission có nguy cơ vượt giới hạn vật lý 6-DOF. Nhấn phím bất kỳ để tiếp tục mô phỏng...');
%     pause; 
% end
% % -------------------------------------------------------------
% % Simulation Time
% T_sim = sys.sim.t_end; 
% t_span = 0:dt:T_sim; 
% N_steps = length(t_span);
% 
% % Init States
% x_true = sys.init.x; 
% 
% % [ĐỐI CHIẾU & SỬA LỖI] Khởi tạo thrust là 8x1 ngay từ đầu để khớp với hệ Coaxial
% act_phys = struct('thrust', zeros(8,1), 'alpha', zeros(4,1), 'beta', zeros(4,1));
% 
% % Init Controller Memory (Khớp 100% với position_ctrl và attitude_ctrl)
% ctrl_state = struct('pos', [], 'att', []);
% for i = 1:3
%     ctrl_state.pos.vel_pid{i}  = struct('prev_error', 0, 'integrator', 0, 'd_filter', 0); 
%     ctrl_state.att.rate_pid{i} = struct('prev_error', 0, 'integrator', 0, 'd_filter', 0); 
% end
% 
% % Init Trajectory Planner State
% traj_state.pos   = sys.init.x(1:3);
% traj_state.vel   = [0;0;0];
% traj_state.euler = sys.init.x(7:9);
% traj_state.rate  = [0;0;0];
% 
% % Init Logger
% history = data_logger('init', N_steps, x_true);
% fprintf('Starting Mission Execution...\n');
% 
% % --- 2. MAIN LOOP ---
% for k = 1:N_steps
%     t = t_span(k);
% 
%     % A. GUIDANCE
%     raw_target = mission_manager(t);
%     [smooth_sp, traj_state] = trajectory_planner(raw_target, dt, traj_limits, traj_state);
% 
%     % B. FLIGHT CONTROL
%     x_est = sensor_model(x_true, sys);
% 
%     % flight_main trả về act_cmd.thrust (8x1) từ WPIN Control Allocation
%     [act_cmd, ctrl_state] = flight_main(x_est, smooth_sp, sys, ctrl_state, ctrl_params, act_phys);
% 
%     % C. PLANT PHYSICS
%     % actuator_dynamics xử lý độ trễ cho mảng 8x1 thrust và 4x1 alpha/beta
%     act_phys = actuator_dynamics(act_cmd, act_phys, sys);
% 
%     % Dynamics Integration (RK4)
%     % [ĐỐI CHIẾU & SỬA LỖI] Truyền TRỰC TIẾP act_phys vào dynamics_6dof
%     % dynamics_6dof đã tự động lấy 1-4 (Top) và 5-8 (Bot) để tính Momen Yaw
%     if k < N_steps
%         [k1,~,~] = dynamics_6dof(t,          x_true,            act_phys, sys);
%         [k2,~,~] = dynamics_6dof(t+0.5*dt, x_true + 0.5*dt*k1, act_phys, sys);
%         [k3,~,~] = dynamics_6dof(t+0.5*dt, x_true + 0.5*dt*k2, act_phys, sys);
%         [k4,~,~] = dynamics_6dof(t+dt,     x_true + dt*k3,     act_phys, sys);
% 
%         x_next = x_true + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
% 
%         % Ground Collision
%         if x_next(3) > 0, x_next(3) = 0; x_next(4:6) = 0; x_next(10:12) = 0; end
%         x_true = x_next;
%     end
% 
%     % D. LOGGING
%     history = data_logger('log', k, x_true, act_phys, act_cmd, smooth_sp, history);
% end
% 
% % --- 3. PLOT SELECTION ---
% active_plots = {'pos', 'att', 'act', '3d'};
% % active_plots = {'pos', 'att', 'act'}; 
% data_logger('plot', history, sys, active_plots);
% 
% fprintf('Simulation Complete.\n');
% ----------------------------------------------------------------------------------------------------------------

% -------------------------------------------------------------------------
% -------------------------------------------------------------------------
% -------------------------------------------------------------------------
% OVAP-X1: Full Mission Simulation
% File Name: main_sim.m
% Position: Root > main_sim.m
% Description: Main Loop with Normal Flight and Sequential Open-Loop Test.
% -------------------------------------------------------------------------
clear; clc; close all;

% =========================================================================
% TÙY CHỌN CHẾ ĐỘ MÔ PHỎNG (CHỌN 1 TRONG 2)
% =========================================================================
% 1: Bay tự động hoàn toàn (Closed-Loop Flight Mode)
% 2: Kiểm chứng Vật lý Vòng hở (Open-Loop Dynamics Test theo trình tự)
SIM_MODE = 3; 
% =========================================================================

% --- 1. CONFIG PATHS ---
addpath(genpath('config'));
addpath(genpath('modules'));     
addpath(genpath('utils'));       

fprintf('Initializing OVAP-X1 Simulation...\n');

% Load Parameters
sys = init_params();  
dt  = sys.sim.dt; 
ctrl_params = controller_params(sys);
traj_limits = init_limits();

% --- KIỂM TRA MISSION FEASIBILITY TRƯỚC KHI BAY ---
if SIM_MODE == 1
    is_safe = mission_feasibility_check(sys, traj_limits);
    if ~is_safe
        warning('Mission có nguy cơ vượt giới hạn vật lý. Nhấn phím bất kỳ để tiếp tục...');
        pause; 
    end
end
% -------------------------------------------------------------
% Simulation Time
T_sim = sys.sim.t_end; 
t_span = 0:dt:T_sim; 
N_steps = length(t_span);

% Init States
x_true = sys.init.x; 

% Khởi tạo thrust là 8x1
act_phys = struct('thrust', zeros(8,1), 'alpha', zeros(4,1), 'beta', zeros(4,1));

% Init Controller Memory
ctrl_state = struct('pos', [], 'att', []);
for i = 1:3
    ctrl_state.pos.vel_pid{i}  = struct('prev_error', 0, 'integrator', 0, 'd_filter', 0); 
    ctrl_state.att.rate_pid{i} = struct('prev_error', 0, 'integrator', 0, 'd_filter', 0); 
end

% Init Trajectory Planner State
traj_state.pos   = sys.init.x(1:3);
traj_state.vel   = [0;0;0];
traj_state.euler = sys.init.x(7:9);
traj_state.rate  = [0;0;0];

% Init Logger
history = data_logger('init', N_steps, x_true);

if SIM_MODE == 1
    fprintf('Starting Mission Execution (Normal Flight)...\n');
else
    fprintf('Starting Sequential Open-Loop Test (Takeoff -> 5m -> Vectoring)...\n');
end

% --- 2. MAIN LOOP ---
for k = 1:N_steps
    t = t_span(k);
    
    % A. GUIDANCE
    raw_target = mission_manager(t);
    [smooth_sp, traj_state] = trajectory_planner(raw_target, dt, traj_limits, traj_state);
    
    % B. FLIGHT CONTROL
    x_est = sensor_model(x_true, sys);
    
    if SIM_MODE == 1
        % --- MODE 1: BỘ ĐIỀU KHIỂN HOẠT ĐỘNG ---
        [act_cmd, ctrl_state] = flight_main(x_est, smooth_sp, sys, ctrl_state, ctrl_params, act_phys);
        
    elseif SIM_MODE == 2
        % --- MODE 2: BÀI TEST CHUỖI SỰ KIỆN VẬT LÝ ---
        act_cmd.thrust = ones(8,1) * 3.5; 
        
        current_alt = -x_true(3);
        
        if current_alt < 5.0
            act_cmd.alpha = [0; 0; 0; 0];
            act_cmd.beta  = [0; 0; 0; 0];
        else
            act_cmd.alpha = [deg2rad(25); deg2rad(20); deg2rad(20); deg2rad(25)];
            act_cmd.beta  = [deg2rad(0); deg2rad(0); deg2rad(0); deg2rad(0)];
        end
        
    elseif SIM_MODE == 3
        % --- MODE 3: BÀN THỬ NGHIỆM ĐÁNH GIÁ TRỌNG SỐ W ---
        W_weight = sys.mass * sys.sim.g; 
        % Yêu cầu Nâng (Fz = -W_weight) và Xoay (Mz = 2.0 N.m)
        tau_des = [0; 0; -W_weight; 0; 0; 2.0]; 
        
        % Gọi bộ não phân bổ lực
        [cmd_thrust, cmd_alpha, cmd_beta] = control_allocation(tau_des, sys, act_phys);
        
        act_cmd.thrust = cmd_thrust;
        act_cmd.alpha  = cmd_alpha;
        act_cmd.beta   = cmd_beta;
        
        % Ép UAV đứng im
        x_true = sys.init.x; 
    end
        
    % C. PLANT PHYSICS
    act_phys = actuator_dynamics(act_cmd, act_phys, sys);
    
    % Dynamics Integration (RK4)
    if SIM_MODE ~= 3 % Nếu không phải Mode 3 thì mới cho bay
        if k < N_steps
            [k1,~,~] = dynamics_6dof(t,          x_true,            act_phys, sys);
            [k2,~,~] = dynamics_6dof(t+0.5*dt, x_true + 0.5*dt*k1, act_phys, sys);
            [k3,~,~] = dynamics_6dof(t+0.5*dt, x_true + 0.5*dt*k2, act_phys, sys);
            [k4,~,~] = dynamics_6dof(t+dt,     x_true + dt*k3,     act_phys, sys);
            
            x_next = x_true + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
            
            % Ground Collision
            if x_next(3) > 0, x_next(3) = 0; x_next(4:6) = 0; x_next(10:12) = 0; end
            x_true = x_next;
        end
    end
    
    % D. LOGGING
    history = data_logger('log', k, x_true, act_phys, act_cmd, smooth_sp, history);
end

% --- 3. PLOT SELECTION ---
% Thêm 'servo' vào danh sách
active_plots = {'pos', 'att', 'act', '3d', 'servo'};
data_logger('plot', history, sys, active_plots);

fprintf('Simulation Complete.\n');