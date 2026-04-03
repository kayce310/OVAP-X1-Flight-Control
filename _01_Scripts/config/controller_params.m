function params = controller_params(sys)
% File Name: controller_params.m
% Description: Tuned for Heavy 2.6kg 650mm Coaxial Tilt-Rotor.

    % --- 1. Position Control ---
    params.pos_P = [1.2; 1.2; 2.0]; % 
    params.max_vel = [2.0; 2.0; 1.5]; % m/s
    
    params.vel_Kp = [3.0; 3.0; 6.0];
    params.vel_Ki = [0.1; 0.1; 0.2];
    params.vel_Kd = [0.2; 0.2; 0.4];
    params.max_acc = [1.5; 1.5; 3.0]; % m/s^2
    
    % --- 2. Attitude Control  ---
    params.att_P = [15.0; 15.0; 8.0]; % Phản ứng cực gắt với sai số góc
    
    % Momen PID: Khung to cần Kp, Kd rất lớn để chống lật
    params.rate_Kp = [25.0; 25.0; 15.0]; 
    params.rate_Ki = [2.0; 2.0; 1.0];
    params.rate_Kd = [3.0; 3.0; 1.5];
    params.max_torque = [10.0; 10.0; 8.0]; % N.m
    
    params.mass = sys.mass;
    params.g = sys.sim.g;
end