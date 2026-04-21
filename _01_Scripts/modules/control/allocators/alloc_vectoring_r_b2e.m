function [thrust, alpha, beta] = alloc_vectoring_r_b2e(acc_cmd_earth, M_body_des, R_b2e, sys, act_phys)
% File Name: alloc_vectoring_r_b2e.m
% Description: Vectoring Allocator sử dụng trực tiếp Ma trận xoay (R_b2e) 
%              để tránh hiện tượng lật trục (Gimbal Lock) của góc Euler ở góc tilt > 90 độ.

    % 1. Tính toán vector lực tổng hợp mong muốn trong hệ tọa độ Trái Đất (Earth Frame)
    mass = sys.mass;
    g = sys.sim.g;
    % Lực yêu cầu = Khối lượng * (Gia tốc mong muốn - Gia tốc trọng trường)
    F_des_earth = mass * (acc_cmd_earth - [0; 0; g]);
    
    % 2. Chiếu vector lực từ Earth về Body Frame
    % R_b2e là ma trận Body -> Earth. Do đó R_b2e' (chuyển vị) là Earth -> Body.
    F_des_body = R_b2e' * F_des_earth;
    
    % 3. Phân bổ lực tịnh tiến cho 4 cụm cánh tay đòn (Arm)
    F_arm_i = F_des_body / 4; 
    
    % 4. Tính toán góc nghiêng Servo (Alpha, Beta)
    % Dùng atan2 để ánh xạ trọn vẹn 360 độ không có điểm mù
    alpha_cmd = atan2(-F_arm_i(1), -F_arm_i(3)); 
    beta_cmd  = atan2(F_arm_i(2), sqrt(F_arm_i(1)^2 + F_arm_i(3)^2));
    
    % 5. Tính toán lực đẩy Motor và Mô-men Yaw (Hệ Coaxial)
    thrust_per_arm = norm(F_arm_i); 
    
    % Tạo Yaw bằng chênh lệch momen xoắn giữa motor trên và dưới
    delta_T_yaw = M_body_des(3) / (4 * sys.motor.tau);
    
    % Khởi tạo mảng đầu ra
    thrust = zeros(8, 1);
    alpha = zeros(4, 1);
    beta = zeros(4, 1);
    
    for i = 1:4
        idx_top = sys.motor.top_idx(i);
        idx_bot = sys.motor.bot_idx(i);
        
        % Lực cơ bản chia đôi, cộng/trừ thành phần Yaw
        thrust(idx_top) = (thrust_per_arm / 2) + delta_T_yaw;
        thrust(idx_bot) = (thrust_per_arm / 2) - delta_T_yaw;
        
        alpha(i) = alpha_cmd;
        beta(i)  = beta_cmd;
    end
end