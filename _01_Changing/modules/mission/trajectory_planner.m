function [setpoint, traj_state] = trajectory_planner(target, dt, constraints, traj_state, planner_mode, sys)
% File Name: trajectory_planner.m
% Position: Root > modules > mission > trajectory_planner.m
% Description: Actuator-Aware Kinematic Planner (Giới hạn Vận tốc, Gia tốc và Độ giật Jerk).

    % [BẢO VỆ]: Nếu gọi hàm mà quên truyền mode, mặc định là bay mượt
    if nargin < 5
        planner_mode = 1; 
    end
    
    % [BẢO VỆ]: Khởi tạo các biến gia tốc nếu traj_state từ bên ngoài chưa có
    if ~isfield(traj_state, 'acc')
        traj_state.acc = zeros(3,1);
        traj_state.rate_dot = zeros(3,1);
    end

    % =====================================================================
    % [MODE 0]: STEP RESPONSE (CHUYÊN DÀNH ĐỂ TUNE PID)
    % =====================================================================
    if planner_mode == 0
        % Ép thẳng tọa độ mục tiêu thành điểm hiện tại (Bỏ qua nội suy)
        traj_state.pos      = target.pos;
        traj_state.vel      = zeros(3,1); 
        traj_state.acc      = zeros(3,1); 
        traj_state.euler    = target.euler;
        traj_state.rate     = zeros(3,1);
        traj_state.rate_dot = zeros(3,1);
        
        setpoint.pos   = target.pos;
        setpoint.vel   = zeros(3,1);
        setpoint.acc   = zeros(3,1);
        setpoint.euler = target.euler;
        setpoint.rate  = zeros(3,1);
        return; % Thoát hàm ngay lập tức
    end

    % =====================================================================
    % [MODE 1]: ACTUATOR-AWARE SMOOTH TRAJECTORY (BAY THỰC TẾ)
    % =====================================================================
    kp_pos_smooth = 2.0; % Hệ số bám đuổi vị trí của Planner
    kp_att_smooth = 3.0; % Hệ số bám đuổi góc của Planner
    
    % --- ĐỒNG BỘ GIỚI HẠN VẬT LÝ TỪ CẤU TRÚC 'sys' ---
    % 1. Gia tốc ngang tối đa (Phụ thuộc vào góc nghiêng ngàm Servo lớn nhất)
    a_max_hw = [sys.sim.g * sin(sys.servo.lim_alpha); 
                sys.sim.g * sin(sys.servo.lim_beta); 
                constraints.a_max(3)]; 
                
    % 2. Độ giật tối đa - Max Jerk (Phụ thuộc vào Tốc độ xoay Servo)
    jerk_max_hw = [sys.sim.g * sys.servo.rate_lim; 
                   sys.sim.g * sys.servo.rate_lim; 
                   50.0]; % Trục Z motor đáp ứng rất nhanh
                   
    % 3. Gia tốc góc lật tối đa (Phụ thuộc vào Tốc độ bẻ Servo)
    alpha_max_hw = [sys.servo.rate_lim; 
                    sys.servo.rate_lim; 
                    constraints.alpha_max(3)];

    % --- 1. Position Smoothing (XYZ) ---
    for i = 1:3
        pos_err = target.pos(i) - traj_state.pos(i);
        
        % P-Controller tính Vận tốc mong muốn (Cắt đuôi ở v_max)
        v_des = kp_pos_smooth * pos_err; 
        v_des = saturate_local(v_des, -constraints.v_max(i), constraints.v_max(i));
        
        % Gia tốc lý thuyết cần thiết
        a_needed = (v_des - traj_state.vel(i)) / dt;
        
        % [RÀNG BUỘC 1]: GIỚI HẠN TỐC ĐỘ XOAY SERVO (JERK LIMITING)
        jerk_req = (a_needed - traj_state.acc(i)) / dt;
        jerk_req = saturate_local(jerk_req, -jerk_max_hw(i), jerk_max_hw(i));
        
        % Tính ra Gia tốc khả thi (Sau khi đã chặn Jerk)
        a_cmd = traj_state.acc(i) + jerk_req * dt;
        
        % [RÀNG BUỘC 2]: GIỚI HẠN GÓC BẺ SERVO (ACCEL LIMITING)
        safe_a_max = min(constraints.a_max(i), a_max_hw(i));
        a_cmd = saturate_local(a_cmd, -safe_a_max, safe_a_max);
        
        % Integrate (Semi-implicit Euler)
        traj_state.acc(i) = a_cmd;
        traj_state.vel(i) = traj_state.vel(i) + traj_state.acc(i) * dt;
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
        
        % [RÀNG BUỘC 3]: GIỚI HẠN GIA TỐC LẬT GÓC
        safe_alpha_max = min(constraints.alpha_max(k), alpha_max_hw(k));
        alpha_cmd = saturate_local(alpha_needed, -safe_alpha_max, safe_alpha_max);
        
        traj_state.rate_dot(k) = alpha_cmd;
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