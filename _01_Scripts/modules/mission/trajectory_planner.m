function [setpoint, traj_state] = trajectory_planner(target, dt, constraints, traj_state)
% File Name: trajectory_planner.m
% Position: Root > modules > mission > trajectory_planner.m
% Description: Generates smooth setpoints respecting kinematic limits.

    % --- [BẢO VỆ] Tham số hóa hệ số nội suy (Tránh Hard-code) ---
    kp_pos_smooth = 2.0; % Hệ số bám đuổi vị trí của Planner
    kp_att_smooth = 3.0; % Hệ số bám đuổi góc của Planner

    % --- 1. Position Smoothing (XYZ) ---
    for i = 1:3
        pos_err = target.pos(i) - traj_state.pos(i);
        
        % P-Controller for desired velocity
        v_des = kp_pos_smooth * pos_err; 
        v_des = saturate_local(v_des, -constraints.v_max(i), constraints.v_max(i));
        
        % Acceleration limit
        a_needed = (v_des - traj_state.vel(i)) / dt;
        a_cmd = saturate_local(a_needed, -constraints.a_max(i), constraints.a_max(i));
        
        % Integrate (Semi-implicit Euler)
        traj_state.vel(i) = traj_state.vel(i) + a_cmd * dt;
        traj_state.pos(i) = traj_state.pos(i) + traj_state.vel(i) * dt;
        
        setpoint.acc(i,1) = a_cmd;
    end
    setpoint.pos = traj_state.pos;
    setpoint.vel = traj_state.vel;
    
    % --- 2. Attitude Smoothing (Euler) ---
    for k = 1:3
        ang_err = target.euler(k) - traj_state.euler(k);
        
        % Wrap Yaw error if needed 
        if k == 3, ang_err = atan2(sin(ang_err), cos(ang_err)); end
        
        rate_des = kp_att_smooth * ang_err;
        rate_des = saturate_local(rate_des, -constraints.w_max(k), constraints.w_max(k));
        
        alpha_needed = (rate_des - traj_state.rate(k)) / dt;
        alpha_cmd = saturate_local(alpha_needed, -constraints.alpha_max(k), constraints.alpha_max(k));
        
        traj_state.rate(k)  = traj_state.rate(k) + alpha_cmd * dt;
        traj_state.euler(k) = traj_state.euler(k) + traj_state.rate(k) * dt;
    end
    setpoint.euler = traj_state.euler;
    setpoint.rate  = traj_state.rate;
end

%% ================= LOCAL FUNCTIONS =================
% Bọc hàm saturate nội bộ để chống crash hệ thống
function v_out = saturate_local(v_in, v_min, v_max)
    v_out = max(min(v_in, v_max), v_min);
end