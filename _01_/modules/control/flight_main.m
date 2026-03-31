function [actuators_cmd, ctrl_state] = flight_main(state_est, setpoints, sys, ctrl_state, params, act_phys)
% File Name: flight_main.m
% Position: Root > modules > control > flight_main.m
% Description: Main control loop. Fully compliant with 6-DOF Decoupling.

    dt = sys.sim.dt;
    
    % Unpack State
    pos_curr_earth = state_est(1:3);   
    vel_curr_body  = state_est(4:6);   
    euler_curr     = state_est(7:9);   
    rate_curr      = state_est(10:12);
    
    % Chuyển đổi vận tốc Body -> Earth (Quan trọng cho Position PID)
    R_b2e = rotation_matrix(euler_curr); 
    vel_curr_earth = R_b2e * vel_curr_body; 
    
    % --- 1. Position Control Loop (Translational Dynamics) ---
    [F_body_des, ctrl_state.pos] = position_ctrl(...
        setpoints.pos, pos_curr_earth, vel_curr_earth, euler_curr, dt, params, ctrl_state.pos);
        
    % --- 2. Attitude Control Loop (Rotational Dynamics) ---
    target_euler = setpoints.euler; 
    [M_body_des, ctrl_state.att] = attitude_ctrl(...
        target_euler, euler_curr, rate_curr, dt, params, ctrl_state.att);
        
    % --- 3. Dynamic Control Allocation (WPIN 16-DOF) ---
    tau_des = [F_body_des; M_body_des];
    [cmd_thrust, cmd_alpha, cmd_beta] = control_allocation(tau_des, sys, act_phys);
    
    % 4. Pack Output (Lưu ý: cmd_thrust hiện tại là 8x1)
    actuators_cmd.thrust = cmd_thrust;
    actuators_cmd.alpha  = cmd_alpha;
    actuators_cmd.beta   = cmd_beta;
end

%% ================= LOCAL FUNCTIONS =================
function R = rotation_matrix(e)
    % Ma trận quay từ Body sang Earth (R_E^B)^T = R_B^E theo chuẩn NED
    ph = e(1); th = e(2); ps = e(3);
    R = [cos(th)*cos(ps), sin(ph)*sin(th)*cos(ps)-cos(ph)*sin(ps), cos(ph)*sin(th)*cos(ps)+sin(ph)*sin(ps);
         cos(th)*sin(ps), sin(ph)*sin(th)*sin(ps)+cos(ph)*cos(ps), cos(ph)*sin(th)*sin(ps)-sin(ph)*cos(ps);
        -sin(th),         sin(ph)*cos(th),                         cos(ph)*cos(th)];
end