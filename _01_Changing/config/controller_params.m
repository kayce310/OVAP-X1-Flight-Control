% function params = controller_params(sys)
% % File Name: controller_params.m
% % Description: Tuned for Heavy 2.6kg 650mm Coaxial Tilt-Rotor.
% 
%     % --- 1. Position Control ---
%     params.pos_P = [1.2; 1.2; 2.0]; % 
%     params.max_vel = [2.0; 2.0; 1.5]; % m/s
% 
%     params.vel_Kp = [3.0; 3.0; 6.0];
%     params.vel_Ki = [0.1; 0.1; 0.2];
%     params.vel_Kd = [0.2; 0.2; 0.4];
%     params.max_acc = [1.5; 1.5; 3.0]; % m/s^2
% 
%     % --- 2. Attitude Control  ---
%     params.att_P = [15.0; 15.0; 8.0]; % Phản ứng cực gắt với sai số góc
% 
%     % Momen PID: Khung to cần Kp, Kd rất lớn để chống lật
%     params.rate_Kp = [25.0; 25.0; 15.0]; 
%     params.rate_Ki = [2.0; 2.0; 1.0];
%     params.rate_Kd = [3.0; 3.0; 1.5];
%     params.max_torque = [10.0; 10.0; 8.0]; % N.m
% 
%     params.mass = sys.mass;
%     params.g = sys.sim.g;
% end
function params = controller_params(sys)
% File Name: controller_params.m
% Position: Root > config > controller_params.m
% Description: Multi-config Tuning Parameters for Benchmarking.

    % =================================================================
    % 1. THÔNG SỐ CHO BỘ ĐIỀU KHIỂN PID + WPIN ALLOCATION (Cũ)
    % (Cần Kp, Kd rất lớn do thuật toán WPIN có xu hướng nén biên độ lệnh)
    % =================================================================
    pid_wpin.pos_P   = [1.2; 1.2; 2.0]; 
    pid_wpin.max_vel = [2.0; 2.0; 1.5]; % m/s
    
    pid_wpin.vel_Kp  = [3.0; 3.0; 6.0];
    pid_wpin.vel_Ki  = [0.1; 0.1; 0.2];
    pid_wpin.vel_Kd  = [0.2; 0.2; 0.4];
    pid_wpin.max_acc = [1.5; 1.5; 3.0]; % m/s^2
    
    pid_wpin.att_P   = [15.0; 15.0; 8.0]; % Phản ứng gắt
    
    pid_wpin.rate_Kp = [25.0; 25.0; 15.0]; 
    pid_wpin.rate_Ki = [2.0;  2.0;  1.0];
    pid_wpin.rate_Kd = [3.0;  3.0;  1.5];
    pid_wpin.max_torque = [10.0; 10.0; 8.0]; % N.m
    
    pid_wpin.mass = sys.mass;
    pid_wpin.g    = sys.sim.g;

    % =================================================================
    % 2. THÔNG SỐ CHO BỘ ĐIỀU KHIỂN PID + ANALYTICAL MIXER (Mới)
    % (Lượng giác 1:1 + Assist -> Cần bộ Gain êm ái hơn để tránh vọt lố)
    % =================================================================
    pid_analytical = pid_wpin; % Kế thừa cấu hình Vị trí từ WPIN
    
    % Giảm Gain Thái độ để chống dao động dư lực
    pid_analytical.att_P   = [8.0; 8.0; 5.0]; 
    
    pid_analytical.rate_Kp = [12.0; 12.0; 8.0]; 
    pid_analytical.rate_Ki = [1.0;  1.0;  0.5];
    pid_analytical.rate_Kd = [1.5;  1.5;  0.8];
    pid_analytical.max_torque = [10.0; 10.0; 8.0]; 

    % =================================================================
    % ĐÓNG GÓI XUẤT RA
    % =================================================================
    params.pid_wpin       = pid_wpin;
    params.pid_analytical = pid_analytical;
end