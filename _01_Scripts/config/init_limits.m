function limits = init_limits()
    % --- 1. Giới hạn Tịnh tiến (Giữ nguyên) ---
    limits.v_max = [2.0; 2.0; 1.5];    
    limits.a_max = [1.0; 1.0; 2.0];    
    
    % --- 2. Giới hạn Quay (Rotation) ---
    % GIẢM XUỐNG để quỹ đạo mượt hơn
    % Vận tốc góc tối đa (rad/s)
    % 45 độ/s (0.8 rad/s) là đủ nhanh cho drone cỡ lớn
    limits.w_max = [0.8; 0.8; 0.8];     
    
    % Gia tốc góc tối đa (rad/s^2)
    limits.alpha_max = [1.5; 1.5; 1.5];
end