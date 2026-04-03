function out = data_logger(mode, varargin)
% File Name: data_logger.m
% Position: Root > modules > visualization > data_logger.m
% Description: Handles data recording and MODULAR plotting using Tabbed Interface.
    % --- [INIT MODE] ---
    if strcmp(mode, 'init')
        N = varargin{1}; x0 = varargin{2};
        hist.time_idx = 0; hist.x = zeros(12, N); hist.x(:,1) = x0;
        hist.pos_des = zeros(3, N); hist.vel_des = zeros(3, N); hist.acc_des = zeros(3, N); hist.euler_des = zeros(3, N);
        
        hist.u_phys_thrust = zeros(8, N); 
        hist.u_phys_alpha  = zeros(4, N); 
        hist.u_phys_beta   = zeros(4, N);
        
        hist.u_cmd_thrust  = zeros(8, N); 
        hist.u_cmd_alpha   = zeros(4, N); 
        hist.u_cmd_beta    = zeros(4, N);
        
        out = hist; return;
    
    % --- [LOG MODE] ---
    elseif strcmp(mode, 'log')
        k = varargin{1}; x_curr = varargin{2}; act_phys = varargin{3}; act_cmd = varargin{4}; sp = varargin{5}; hist = varargin{6}; 
        if k > size(hist.x, 2), return; end
        hist.x(:, k) = x_curr;
        hist.pos_des(:, k) = sp.pos; hist.vel_des(:, k) = sp.vel; if isfield(sp, 'acc'), hist.acc_des(:, k) = sp.acc; end; hist.euler_des(:, k) = sp.euler;
        
        hist.u_phys_thrust(:, k) = act_phys.thrust; hist.u_phys_alpha(:, k) = act_phys.alpha; hist.u_phys_beta(:, k) = act_phys.beta;
        hist.u_cmd_thrust(:, k)  = act_cmd.thrust;  hist.u_cmd_alpha(:, k)  = act_cmd.alpha;  hist.u_cmd_beta(:, k)  = act_cmd.beta;
        out = hist; return;
    end
    
    % --- [PLOT MODE] ---
    if strcmp(mode, 'plot')
        hist = varargin{1}; sys = varargin{2};
        if length(varargin) > 2, active_plots = varargin{3}; else, active_plots = {'pos', 'att', 'act', '3d'}; end
        
        end_idx = find(hist.x(1,:) ~= 0, 1, 'last');
        if isempty(end_idx), end_idx = length(hist.x); end
        t = (0:end_idx-1) * sys.sim.dt;
        
        % Slice arrays
        hist.x = hist.x(:, 1:end_idx); 
        hist.pos_des = hist.pos_des(:, 1:end_idx); 
        hist.euler_des = hist.euler_des(:, 1:end_idx);
        hist.u_phys_alpha = hist.u_phys_alpha(:, 1:end_idx); 
        hist.u_phys_beta = hist.u_phys_beta(:, 1:end_idx); 
        hist.u_phys_thrust = hist.u_phys_thrust(:, 1:end_idx);
        
        % TẠO CỬA SỔ TỔNG VÀ GROUP TAB ĐỂ GOM BIỂU ĐỒ
        hFigMain = figure('Name', 'OVAP-X1 Analytics Dashboard', 'Color', 'w', 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]);
        tgroup = uitabgroup('Parent', hFigMain);
        
        % --- MODULE 1: POSITION ---
        if ismember('pos', active_plots)
            tabPos = uitab('Parent', tgroup, 'Title', '1. Position');
            ax1 = axes('Parent', tabPos, 'Position', [0.1 0.7 0.85 0.25]);
            plot(ax1, t, hist.pos_des(1,:), 'r--'); hold(ax1, 'on'); plot(ax1, t, hist.x(1,:), 'r-');
            grid(ax1, 'on'); ylabel(ax1, 'North (m)'); title(ax1, 'Position Tracking'); legend(ax1, 'Setpoint', 'Response');
            
            ax2 = axes('Parent', tabPos, 'Position', [0.1 0.4 0.85 0.25]);
            plot(ax2, t, hist.pos_des(2,:), 'g--'); hold(ax2, 'on'); plot(ax2, t, hist.x(2,:), 'g-');
            grid(ax2, 'on'); ylabel(ax2, 'East (m)'); legend(ax2, 'Setpoint', 'Response');
            
            ax3 = axes('Parent', tabPos, 'Position', [0.1 0.1 0.85 0.25]);
            plot(ax3, t, hist.pos_des(3,:), 'b--'); hold(ax3, 'on'); plot(ax3, t, hist.x(3,:), 'b-');
            grid(ax3, 'on'); ylabel(ax3, 'Down (m)'); xlabel(ax3, 'Time (s)'); legend(ax3, 'Setpoint', 'Response');
        end
        
        % --- MODULE 2: ATTITUDE ---
        if ismember('att', active_plots)
            tabAtt = uitab('Parent', tgroup, 'Title', '2. Attitude');
            ax1 = axes('Parent', tabAtt, 'Position', [0.1 0.7 0.85 0.25]);
            plot(ax1, t, rad2deg(hist.euler_des(1,:)), 'r--'); hold(ax1, 'on'); plot(ax1, t, rad2deg(hist.x(7,:)), 'r-');
            grid(ax1, 'on'); ylabel(ax1, 'Roll (deg)'); title(ax1, 'Attitude Tracking'); legend(ax1, 'Setpoint', 'Response');
            
            ax2 = axes('Parent', tabAtt, 'Position', [0.1 0.4 0.85 0.25]);
            plot(ax2, t, rad2deg(hist.euler_des(2,:)), 'g--'); hold(ax2, 'on'); plot(ax2, t, rad2deg(hist.x(8,:)), 'g-');
            grid(ax2, 'on'); ylabel(ax2, 'Pitch (deg)'); legend(ax2, 'Setpoint', 'Response');
            
            ax3 = axes('Parent', tabAtt, 'Position', [0.1 0.1 0.85 0.25]);
            plot(ax3, t, rad2deg(hist.euler_des(3,:)), 'b--'); hold(ax3, 'on'); plot(ax3, t, rad2deg(hist.x(9,:)), 'b-');
            grid(ax3, 'on'); ylabel(ax3, 'Yaw (deg)'); xlabel(ax3, 'Time (s)'); legend(ax3, 'Setpoint', 'Response');
        end
        
        % --- MODULE 3: ACTUATORS ---
        if ismember('act', active_plots)
            tabAct = uitab('Parent', tgroup, 'Title', '3. Actuators');
            ax1 = axes('Parent', tabAct, 'Position', [0.1 0.7 0.85 0.25]);
            plot(ax1, t, rad2deg(hist.u_phys_alpha(1,:)), 'r', 'LineWidth', 1.2); hold(ax1, 'on');
            plot(ax1, t, rad2deg(hist.u_phys_alpha(2,:)), 'g', 'LineWidth', 1.2);
            plot(ax1, t, rad2deg(hist.u_phys_alpha(3,:)), 'b', 'LineWidth', 1.2);
            plot(ax1, t, rad2deg(hist.u_phys_alpha(4,:)), 'k', 'LineWidth', 1.2);
            grid(ax1, 'on'); ylabel(ax1, 'Alpha (deg)'); title(ax1, 'Servo Tilt: Pitch'); 
            legend(ax1, 'M1 (FR)', 'M2 (RL)', 'M3 (FL)', 'M4 (RR)', 'Location', 'best');
            
            ax2 = axes('Parent', tabAct, 'Position', [0.1 0.4 0.85 0.25]);
            plot(ax2, t, rad2deg(hist.u_phys_beta(1,:)), 'r', 'LineWidth', 1.2); hold(ax2, 'on');
            plot(ax2, t, rad2deg(hist.u_phys_beta(2,:)), 'g', 'LineWidth', 1.2);
            plot(ax2, t, rad2deg(hist.u_phys_beta(3,:)), 'b', 'LineWidth', 1.2);
            plot(ax2, t, rad2deg(hist.u_phys_beta(4,:)), 'k', 'LineWidth', 1.2);
            grid(ax2, 'on'); ylabel(ax2, 'Beta (deg)'); title(ax2, 'Servo Tilt: Roll');
            
            ax3 = axes('Parent', tabAct, 'Position', [0.1 0.1 0.85 0.25]);
            plot(ax3, t, hist.u_phys_thrust(1,:), 'r-', 'LineWidth', 1); hold(ax3, 'on'); 
            plot(ax3, t, hist.u_phys_thrust(2,:), 'g-', 'LineWidth', 1);
            plot(ax3, t, hist.u_phys_thrust(3,:), 'b-', 'LineWidth', 1); 
            plot(ax3, t, hist.u_phys_thrust(4,:), 'k-', 'LineWidth', 1);
            plot(ax3, t, hist.u_phys_thrust(5,:), 'r--', 'LineWidth', 1); 
            plot(ax3, t, hist.u_phys_thrust(6,:), 'g--', 'LineWidth', 1);
            plot(ax3, t, hist.u_phys_thrust(7,:), 'b--', 'LineWidth', 1); 
            plot(ax3, t, hist.u_phys_thrust(8,:), 'k--', 'LineWidth', 1);
            grid(ax3, 'on'); ylabel(ax3, 'Force (N)'); title(ax3, 'Coaxial Thrust (Solid: Top, Dashed: Bot)'); xlabel(ax3, 'Time (s)');
        end

        % --- MODULE 4: SERVO ANALYSIS ---
        if ismember('servo', active_plots)
            tabServo = uitab('Parent', tgroup, 'Title', '4. Servo Analysis (W Matrix)');
            
            N_len = size(hist.u_phys_alpha, 2);
            t_servo = (0:N_len-1) * sys.sim.dt; 
            alpha_deg = rad2deg(hist.u_phys_alpha);
            alpha_vel = [zeros(4,1), diff(alpha_deg, 1, 2) ./ sys.sim.dt];
            diff_thrust = hist.u_phys_thrust(1:4, :) - hist.u_phys_thrust(5:8, :);
            
            ax1 = axes('Parent', tabServo, 'Position', [0.1 0.7 0.85 0.25]);
            plot(ax1, t_servo, alpha_deg, 'LineWidth', 1.5);
            title(ax1, 'Biên độ góc xoay Servo \alpha (Độ rướn cơ khí)', 'FontWeight', 'bold');
            ylabel(ax1, 'Angle (deg)'); grid(ax1, 'on');
            legend(ax1, 'M1', 'M2', 'M3', 'M4', 'Location', 'best');
            
            ax2 = axes('Parent', tabServo, 'Position', [0.1 0.4 0.85 0.25]);
            plot(ax2, t_servo, alpha_vel, 'LineWidth', 1.5);
            title(ax2, 'Vận tốc xoay Servo d\alpha/dt (Độ gắt của lệnh)', 'FontWeight', 'bold');
            ylabel(ax2, 'Velocity (deg/s)'); grid(ax2, 'on');
            
            ax3 = axes('Parent', tabServo, 'Position', [0.1 0.1 0.85 0.25]);
            plot(ax3, t_servo, diff_thrust, 'LineWidth', 1.5);
            title(ax3, 'Chênh lệch lực đẩy Trên-Dưới (Độ hao phí tiêu thụ điện ESC)', 'FontWeight', 'bold');
            ylabel(ax3, '\Delta Thrust (N)'); xlabel(ax3, 'Time (s)'); grid(ax3, 'on');
        end
        
        % --- MODULE 5: 3D VISUALIZATION (GIỮ NGUYÊN LÀ CỬA SỔ RIÊNG) ---
        if ismember('3d', active_plots)
            if exist('visualize_3d', 'file')
                fprintf('Launching 3D Visualizer...\n');
                % Cửa sổ 3D vẫn mở thành cửa sổ riêng biệt để bạn dễ tương tác Rotate/Pan/Zoom
                visualize_3d(hist, sys, t);
            else
                warning('visualize_3d.m not found. Skipping 3D replay.');
            end
        end
        
        out = [];
    end
end