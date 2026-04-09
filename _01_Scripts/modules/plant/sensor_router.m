function meas = sensor_router(kinematics_mode, x_true, use_noise)
% File Name: sensor_router.m

    if strcmp(kinematics_mode, 'dcm')
        % 1. Dịch 18-State -> 12-State Euler
        R_eb = reshape(x_true(7:15), 3, 3);
        
        % [BẢN VÁ LỖI]: Trực giao hóa nhưng ÉP BUỘC ĐỊNH THỨC = +1 (Chống lật gương)
        [U, ~, V] = svd(R_eb); 
        det_UV = det(U * V');
        R_eb = U * diag([1, 1, det_UV]) * V'; 
        
        % Trích xuất Euler tiêu chuẩn
        pitch = asin(max(min(-R_eb(3,1), 1), -1));
        if abs(cos(pitch)) > 1e-4
            roll = atan2(R_eb(3,2), R_eb(3,3));
            yaw  = atan2(R_eb(2,1), R_eb(1,1));
        else
            roll = 0; 
            yaw  = atan2(-R_eb(1,2), R_eb(2,2));
        end
        
        x_legacy = [x_true(1:6); roll; pitch; yaw; x_true(16:18)];
        
        % 2. Tái sử dụng hàm sensor cũ để bơm nhiễu
        meas = sensor_model(x_legacy, use_noise);
    else
        % Hệ cũ (Euler 12-State)
        meas = sensor_model(x_true, use_noise);
    end
end