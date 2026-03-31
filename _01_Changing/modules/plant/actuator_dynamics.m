function next_act = actuator_dynamics(cmd_act, prev_act, sys)
% File Name: actuator_dynamics.m
% Position: Root > modules > plant > actuator_dynamics.m
% Description: Simulates motor lag and servo dynamics (rate limit & lag).
%              Added defensive checks for 8-motor coaxial array expansion.

    dt = sys.sim.dt;
    
    % --- [BẢO VỆ 1: ĐỒNG BỘ KÍCH THƯỚC MA TRẬN] ---
    % Nếu prev_act khởi tạo ở main_sim là 4x1, nhưng cmd_act từ allocation là 8x1
    % Tự động mở rộng prev_act thành 8x1 để tránh lỗi Dimension Mismatch
    if length(prev_act.thrust) ~= length(cmd_act.thrust)
        prev_act.thrust = zeros(size(cmd_act.thrust));
    end
    
    % --- [BẢO VỆ 2: AN TOÀN THAM SỐ (Fallback Values)] ---
    tau_m = 0.03; 
    if isfield(sys.motor, 'tau'), tau_m = sys.motor.tau; end
    
    tau_s = 0.05; % Thời hằng Servo (giả định 50ms nếu chưa có)
    if isfield(sys.servo, 'tau'), tau_s = sys.servo.tau; end
    
    rate_lim = deg2rad(300); % Tốc độ xoay tối đa (giả định 300 độ/s nếu chưa có)
    if isfield(sys.servo, 'rate_lim'), rate_lim = sys.servo.rate_lim; end
    
    % --- 1. Motor Dynamics (First Order Lag) ---
    alpha_m = dt / (tau_m + dt);
    
    thrust_next = prev_act.thrust + alpha_m * (cmd_act.thrust - prev_act.thrust);
    thrust_next = max(0, thrust_next); % Clamp Thrust >= 0
    
    % --- 2. Servo Dynamics (Lag + Rate Limit) ---
    alpha_s = dt / (tau_s + dt);
    
    % Update Alpha (Tilt Body)
    alpha_next = update_servo_channel(cmd_act.alpha, prev_act.alpha, alpha_s, rate_lim, dt);
    
    % Update Beta (Tilt Arm)
    beta_next = update_servo_channel(cmd_act.beta, prev_act.beta, alpha_s, rate_lim, dt);
    
    % Clamp Angles using local utils
    alpha_next = saturate(alpha_next, -sys.servo.lim_alpha, sys.servo.lim_alpha);
    beta_next  = saturate(beta_next,  -sys.servo.lim_beta,  sys.servo.lim_beta);
    
    % --- Pack Output ---
    next_act.thrust = thrust_next;
    next_act.alpha  = alpha_next;
    next_act.beta   = beta_next;
end

%% ================= LOCAL FUNCTIONS =================
function angle_out = update_servo_channel(cmd, prev, alpha, limit, dt)
    % 1. Lag
    ideal_next = prev + alpha * (cmd - prev);
    
    % 2. Rate Limit Logic
    delta = ideal_next - prev;
    max_step = limit * dt;
    
    delta = saturate(delta, -max_step, max_step);
    angle_out = prev + delta;
end

% Bổ sung hàm saturate dạng Local để chống lỗi Missing Function
function v_out = saturate(v_in, v_min, v_max)
    v_out = max(min(v_in, v_max), v_min);
end