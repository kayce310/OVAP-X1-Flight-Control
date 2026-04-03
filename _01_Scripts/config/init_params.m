function sys = init_params()
% File Name: init_params.m
% Position: Root > config > init_params.m
% Description: Updated parameters matched with OVAP-X1 Hardware Specs 
% (Wheelbase 650mm, 7-inch props, T-Motor F90, 6S 5000mAh).
% Coaxial aerodynamics (h_eff, lambda) and PX4 motor directions integrated.

    % --- 1. Simulation Settings ---
    sys.sim.dt = 0.002;        
    sys.sim.g = 9.81;
    sys.sim.t_end = 20.0;
    
    % --- 2. Khối lượng & Quán tính (Cập nhật theo thực tế) ---
    % Ước tính: Frame 800g + 8xMotor(400g) + Pin 6S 5000(800g) + Servos(400g) + FC/ESC/GPS(200g)
    sys.mass = 2.6; % Tổng khối lượng khoảng 2.6 kg
    
    % Quán tính (Tăng lên do khung 650mm to hơn)
    sys.Ixx = 0.085;  
    sys.Iyy = 0.150;  
    sys.Izz = 0.210;
    sys.J = diag([sys.Ixx, sys.Iyy, sys.Izz]);
    sys.invJ = inv(sys.J);
    
    % --- 3. Hình học (Cập nhật Wheelbase 650mm) ---
    % Wheelbase 0.65m -> Bán kính r = 0.325m.
    % Tọa độ X, Y = 0.325 * cos(45 độ) = 0.23m
    sys.geo.dist_x = 0.230; 
    sys.geo.dist_y = 0.230; 
    sys.geo.z_offset = 0.0;
    
    % Vector vị trí ngàm xoay Pivot [x, y, z]
    sys.geo.r = [
         sys.geo.dist_x,  sys.geo.dist_y, sys.geo.z_offset;  % 1: FR
        -sys.geo.dist_x, -sys.geo.dist_y, sys.geo.z_offset;  % 2: RL
         sys.geo.dist_x, -sys.geo.dist_y, sys.geo.z_offset;  % 3: FL
        -sys.geo.dist_x,  sys.geo.dist_y, sys.geo.z_offset   % 4: RR
    ];
    
    % --- 4. Actuators ---
    % Đặc tính Động cơ và Cơ cấu Đồng trục
    % T-Motor F90 1300KV + 7-inch prop + 6S: Max thrust khoảng ~1.8kg/motor (17.6 N)
    sys.motor.max_thrust = 17.6;     
    sys.motor.min_thrust = 0.0;      
    sys.motor.tau = 0.03;           % c_q: Hằng số momen/lực đẩy
    
    % [CẬP NHẬT THEO FCD3 & R-CRM]: 2 hệ số suy giảm riêng biệt cho tầng dưới
    sys.motor.coax_eff        = 0.85; % eta_coax (p1): Suy giảm Lực đẩy (Thrust loss)
    sys.motor.coax_torque_eff = 0.90; % lambda (q1): Suy giảm Momen xoắn (Torque loss)
    
    % [CẬP NHẬT THEO FCD3]: Khoảng cách từ ngàm bẻ đến mặt phẳng đĩa rotor
    sys.geo.h_top =  0.02; % Cánh quạt trên cách ngàm 2cm lên trên
    sys.geo.h_bot = -0.02; % Cánh quạt dưới cách ngàm 2cm xuống dưới
    % Tâm lực đẩy hiệu dụng (Thay thế cho h_prop cũ)
    sys.geo.h_eff = (sys.geo.h_top + sys.motor.coax_eff * sys.geo.h_bot) / (1 + sys.motor.coax_eff);
    
    % Chiều quay động cơ: 1 (CCW), -1 (CW) chuẩn PX4
    sys.motor.dir_top = [ 1;  1; -1; -1]; % FR, RL, FL, RR
    sys.motor.dir_bot = [-1; -1;  1;  1];
    
    % Giới hạn Servo KST 
    sys.servo.lim_alpha = deg2rad(90); 
    sys.servo.lim_beta  = deg2rad(90); 
    sys.servo.rate_lim  = 15.0;        
    sys.servo.tau       = 0.015;       
    
    % Bù trừ chiều xoay cơ khí (Lắp ngược đối xứng gương)
    sys.servo.dir_alpha = [1; -1; -1; 1]; 
    sys.servo.dir_beta  = [1; -1; -1; 1]; 
    
    % --- 5. Initial Conditions ---
    sys.init.x = zeros(12, 1);
end