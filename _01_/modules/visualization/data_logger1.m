function out = data_logger1(mode, varargin)
    % DATA_LOGGER - Professional 3D Visualization for OVAP-X1
    % Features: Play/Replay controls, Distinct Nose Marker, High-Fidelity Model
    
    % [INIT MODE]
    if strcmp(mode, 'init')
        N = varargin{1}; x0 = varargin{2};
        hist.time_idx = 0;
        hist.x = zeros(12, N); hist.x(:,1) = x0;
        hist.u_phys_thrust = zeros(4, N); hist.u_phys_alpha = zeros(4, N); hist.u_phys_beta = zeros(4, N);
        hist.u_cmd_thrust = zeros(4, N); hist.u_cmd_alpha = zeros(4, N); hist.u_cmd_beta = zeros(4, N);
        out = hist; return;
    
    % [LOG MODE]
    elseif strcmp(mode, 'log')
        k = varargin{1}; x_curr = varargin{2}; act_phys = varargin{3}; act_cmd = varargin{4}; hist = varargin{5};
        hist.x(:, k) = x_curr;
        hist.u_phys_thrust(:, k) = act_phys.thrust; hist.u_phys_alpha(:, k) = act_phys.alpha; hist.u_phys_beta(:, k) = act_phys.beta;
        hist.u_cmd_thrust(:, k) = act_cmd.thrust; hist.u_cmd_alpha(:, k) = act_cmd.alpha; hist.u_cmd_beta(:, k) = act_cmd.beta;
        out = hist; return;
    end

    % [PLOT MODE]
    if strcmp(mode, 'plot')
        hist = varargin{1};
        sys = varargin{2};
        
        active_plots = {...
            '3d_interactive',...
            % 'translation',...
            % 'attitude',...
            % 'actuators_tilt'...
            };
        
        N_valid = length(hist.x);
        t_arr = (0:N_valid-1) * sys.sim.dt;
        c_x = [0.85 0.33 0.1]; c_y = [0.47 0.67 0.19]; c_z = [0.0 0.45 0.74];

        % --- CASE 1: INTERACTIVE 3D PLAYER ---
        if ismember('3d_interactive', active_plots)
            visualize_interactive_3d(hist, sys, t_arr);
        end

        % --- 2D CHARTS ---
        if ismember('translation', active_plots)
            figure('Name', 'Translational Dynamics', 'Color', 'w');
            ax(1) = subplot(2,1,1); plot(t_arr, hist.x(1,:), 'Color', c_x); hold on; plot(t_arr, hist.x(2,:), 'Color', c_y); plot(t_arr, hist.x(3,:), 'Color', c_z);
            grid on; ylabel('Pos (m)'); legend('N', 'E', 'D'); title('Position');
            ax(2) = subplot(2,1,2); plot(t_arr, hist.x(4,:), 'Color', c_x); hold on; plot(t_arr, hist.x(5,:), 'Color', c_y); plot(t_arr, hist.x(6,:), 'Color', c_z);
            grid on; ylabel('Vel (m/s)'); legend('u', 'v', 'w'); linkaxes(ax, 'x');
        end
        
        if ismember('actuators_tilt', active_plots)
            figure('Name', 'Servo Angles', 'Color', 'w');
            subplot(2,1,1); plot(t_arr, rad2deg(hist.u_phys_alpha(1,:)), 'r'); hold on; plot(t_arr, rad2deg(hist.u_phys_alpha(2,:)), 'b');
            grid on; ylabel('Deg'); title('Alpha (Body Tilt)'); legend('Arm1', 'Arm2');
            subplot(2,1,2); plot(t_arr, rad2deg(hist.u_phys_beta(1,:)), 'r'); hold on; plot(t_arr, rad2deg(hist.u_phys_beta(2,:)), 'b');
            grid on; ylabel('Deg'); title('Beta (Arm Tilt)');
        end
        
        out = [];
    end
end

% =========================================================================
% HÀM VẼ 3D TƯƠNG TÁC (CÓ NÚT PLAY/REPLAY)
% =========================================================================
function visualize_interactive_3d(hist, sys, t_arr)
    hFig = figure('Name', 'OVAP-X1 Flight Replay', 'Color', 'w', 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]);
    
    % --- 1. MÔI TRƯỜNG 3D ---
    plot3(hist.x(2,:), hist.x(1,:), -hist.x(3,:), 'b-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1); 
    hold on; grid on; axis equal; box on;
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Altitude (m)');
    view(45, 30); rotate3d on;
    camlight('headlight'); lighting gouraud; material dull;

    % --- 2. UI CONTROLS (Nút bấm & Thanh trượt) ---
    % Trạng thái Play
    gui_state.is_playing = false;
    gui_state.play_speed = 1; % Tốc độ nhân (1x)
    
    % Slider
    hSlider = uicontrol('Parent', hFig, 'Style', 'slider', 'Units', 'normalized', 'Position', [0.15, 0.05, 0.7, 0.03], ...
        'Min', 1, 'Max', length(t_arr), 'Value', 1, 'Callback', @cb_slider);
        
    % Time Text
    hText = uicontrol('Parent', hFig, 'Style', 'text', 'Units', 'normalized', 'Position', [0.45, 0.09, 0.1, 0.03], ...
        'String', '0.00 s', 'FontSize', 11, 'BackgroundColor', 'w', 'FontWeight', 'bold');

    % Nút PLAY / PAUSE
    hBtnPlay = uicontrol('Parent', hFig, 'Style', 'pushbutton', 'String', '▶ PLAY', ...
        'Units', 'normalized', 'Position', [0.05, 0.05, 0.08, 0.05], ...
        'FontSize', 10, 'FontWeight', 'bold', 'Callback', @cb_play);
        
    % Nút REPLAY
    hBtnReplay = uicontrol('Parent', hFig, 'Style', 'pushbutton', 'String', '↺ REPLAY', ...
        'Units', 'normalized', 'Position', [0.87, 0.05, 0.08, 0.05], ...
        'FontSize', 10, 'Callback', @cb_replay);

    % --- 3. KHỞI TẠO ĐỐI TƯỢNG ĐỒ HỌA ---
    % Thân (Fuselage) - Xám nhạt
    [v_fuselage, f_fuselage] = create_rounded_box(0.6, 0.16, 0.12, 0.3);
    hFuselage = patch('Vertices', v_fuselage, 'Faces', f_fuselage, 'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceLighting', 'gouraud');

    % Kính lái (Cockpit) - Đen bóng
    [v_cp, f_cp] = create_rounded_box(0.12, 0.10, 0.04, 0.5);
    % Dịch chuyển tương đối so với tâm
    v_cp(:,3) = v_cp(:,3) + 0.065; % Lên trên
    v_cp(:,1) = v_cp(:,1) + 0.15;  % Ra trước
    hCockpit = patch('Vertices', v_cp, 'Faces', f_cp, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.9);

    % [MỚI] Mũi định hướng (Nose Marker) - Cam nổi bật
    % Hình nón cụt hoặc khối hộp nhỏ ở mũi
    [v_nose, f_nose] = create_rounded_box(0.05, 0.08, 0.06, 0.2);
    v_nose(:,1) = v_nose(:,1) + 0.31; % Gắn vào cực mũi (L/2 + xíu)
    hNose = patch('Vertices', v_nose, 'Faces', f_nose, 'FaceColor', [1.0 0.5 0.0], 'EdgeColor', 'none'); % Cam

    % [MỚI] Vạch lưng (Spine Marker) - Để biết đâu là mặt trên
    [v_spine, f_spine] = create_rounded_box(0.3, 0.02, 0.01, 0.1);
    v_spine(:,3) = v_spine(:,3) + 0.061; % Nổi lên mặt lưng
    v_spine(:,1) = v_spine(:,1) - 0.1;   % Lùi về sau cockpit
    hSpine = patch('Vertices', v_spine, 'Faces', f_spine, 'FaceColor', [1.0 0.5 0.0], 'EdgeColor', 'none');

    % Arms & Props
    for i=1:4
        hArms(i).U_Shape = plot3(nan, nan, nan, 'k-', 'LineWidth', 2);
        
        [v_disk, f_disk] = create_disk(0.12, 20);
        hArms(i).Prop = patch('Vertices', v_disk, 'Faces', f_disk, 'FaceColor', [0.8 0.0 0.0], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
        
        hArms(i).Force = quiver3(nan, nan, nan, nan, nan, nan, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    end
    
    % Lưu geometry gốc
    base_geo.fuselage = v_fuselage;
    base_geo.cockpit  = v_cp;
    base_geo.nose     = v_nose;
    base_geo.spine    = v_spine;
    base_geo.disk     = v_disk;

    % Cập nhật frame đầu tiên
    update_visuals(1);

    % --- CALLBACK FUNCTIONS ---
    
    function cb_slider(src, ~)
        % Khi người dùng kéo thanh trượt
        gui_state.is_playing = false; % Dừng auto-play
        set(hBtnPlay, 'String', '▶ PLAY');
        idx = round(get(src, 'Value'));
        update_visuals(idx);
    end

    function cb_play(~, ~)
        % Khi nhấn nút Play/Pause
        gui_state.is_playing = ~gui_state.is_playing;
        if gui_state.is_playing
            set(hBtnPlay, 'String', '⏸ PAUSE');
            run_playback_loop();
        else
            set(hBtnPlay, 'String', '▶ PLAY');
        end
    end

    function cb_replay(~, ~)
        % Khi nhấn Replay
        gui_state.is_playing = true;
        set(hBtnPlay, 'String', '⏸ PAUSE');
        set(hSlider, 'Value', 1);
        run_playback_loop();
    end

    function run_playback_loop()
        % Vòng lặp phát lại
        while gui_state.is_playing && isvalid(hFig)
            curr_idx = round(get(hSlider, 'Value'));
            
            % Tăng index (tốc độ phụ thuộc vào sức mạnh máy tính & drawnow)
            step = 5; % Nhảy 5 frame mỗi lần vẽ để mượt hơn (fast forward nhẹ)
            next_idx = curr_idx + step;
            
            if next_idx >= length(t_arr)
                next_idx = length(t_arr);
                gui_state.is_playing = false;
                set(hBtnPlay, 'String', '▶ PLAY');
            end
            
            set(hSlider, 'Value', next_idx);
            update_visuals(next_idx);
            drawnow limitrate; % Vẽ lại màn hình
        end
    end

    function update_visuals(idx)
        % Hàm cập nhật vị trí các khối 3D
        if idx < 1, idx = 1; end
        if idx > length(t_arr), idx = length(t_arr); end
        
        t_curr = t_arr(idx);
        set(hText, 'String', sprintf('%.2f s', t_curr));
        
        % Lấy dữ liệu
        pos_ned = hist.x(1:3, idx);
        euler   = hist.x(7:9, idx);
        alphas  = hist.u_phys_alpha(:, idx);
        betas   = hist.u_phys_beta(:, idx);
        thrusts = hist.u_phys_thrust(:, idx);
        
        R_b2e = get_rotation_matrix(euler);
        p0 = [pos_ned(2); pos_ned(1); -pos_ned(3)]; % ENU
        ned2plot = @(v) [v(2); v(1); -v(3)];
        
        % 1. Cập nhật Thân & Các phần gắn liền (Cockpit, Nose, Spine)
        set(hFuselage, 'Vertices', transform_verts(base_geo.fuselage, R_b2e, p0, ned2plot));
        set(hCockpit,  'Vertices', transform_verts(base_geo.cockpit,  R_b2e, p0, ned2plot));
        set(hNose,     'Vertices', transform_verts(base_geo.nose,     R_b2e, p0, ned2plot));
        set(hSpine,    'Vertices', transform_verts(base_geo.spine,    R_b2e, p0, ned2plot));
        
        % 2. Cập nhật Cánh tay (Arms)
        lx = sys.geo.lx; ly = sys.geo.ly;
        shoulders = [ ly, lx, 0; -ly, -lx, 0; ly, -lx, 0; -ly, lx, 0 ]';
        r_shroud = 0.13; 
        
        for k=1:4
            s_ned = R_b2e * shoulders(:,k);
            s_plot = p0 + ned2plot(s_ned);
            
            alpha = alphas(k); beta = betas(k);
            R_alpha = [cos(alpha) 0 sin(alpha); 0 1 0; -sin(alpha) 0 cos(alpha)];
            R_beta  = [1 0 0; 0 cos(beta) -sin(beta); 0 sin(beta) cos(beta)];
            R_arm = R_b2e * R_alpha;
            R_prop = R_b2e * R_alpha * R_beta;
            
            % Khung U
            theta_arc = linspace(-pi/4, 5*pi/4, 25);
            u_pts_local = [r_shroud*cos(theta_arc); r_shroud*sin(theta_arc); zeros(1,25)];
            % Xoay vector cục bộ rồi cộng vào gốc (s_ned)
            % Cách tính đúng: P_global = P_shoulder + R_arm * P_local
            u_X = zeros(1,25); u_Y = zeros(1,25); u_Z = zeros(1,25);
            for m=1:25
                vec_ned = s_ned + R_arm * u_pts_local(:,m);
                pt = p0 + ned2plot(vec_ned - s_ned); % Hack dịch chuyển tương đối
                % Cách chuẩn:
                vec_abs = R_arm * u_pts_local(:,m);
                vec_final = s_plot + ned2plot(vec_abs);
                u_X(m) = vec_final(1); u_Y(m) = vec_final(2); u_Z(m) = vec_final(3);
            end
            set(hArms(k).U_Shape, 'XData', [s_plot(1) u_X s_plot(1)], 'YData', [s_plot(2) u_Y s_plot(2)], 'ZData', [s_plot(3) u_Z s_plot(3)]);
            
            % Đĩa Cánh Quạt
            set(hArms(k).Prop, 'Vertices', transform_verts(base_geo.disk, R_prop, s_plot, ned2plot));
            
            % Vector Lực
            f_vec_plot = ned2plot(R_prop * [0;0;-1]);
            scale_F = 0.05 * thrusts(k);
            set(hArms(k).Force, 'XData', s_plot(1), 'YData', s_plot(2), 'ZData', s_plot(3), ...
                'UData', f_vec_plot(1)*scale_F, 'VData', f_vec_plot(2)*scale_F, 'WData', f_vec_plot(3)*scale_F);
        end
    end
end

% --- HELPER FUNCTIONS ---
function [v, f] = create_rounded_box(L, W, H, n_curve)
    % Super-ellipsoid mesh
    N = 12;
    eta = linspace(-pi/2, pi/2, N);
    omega = linspace(-pi, pi, N);
    [ETA, OMEGA] = meshgrid(eta, omega);
    ce = sign(cos(ETA)).*abs(cos(ETA)).^n_curve;
    se = sign(sin(ETA)).*abs(sin(ETA)).^n_curve;
    co = sign(cos(OMEGA)).*abs(cos(OMEGA)).^n_curve;
    so = sign(sin(OMEGA)).*abs(sin(OMEGA)).^n_curve;
    X = (L/2) * ce .* co; Y = (W/2) * ce .* so; Z = (H/2) * se;
    [f, v] = surf2patch(X, Y, Z, 'triangles');
end

function [v, f] = create_disk(R, N)
    theta = linspace(0, 2*pi, N+1)'; theta(end) = [];
    v = [0 0 0; R*cos(theta), R*sin(theta), zeros(N,1)];
    f = zeros(N, 3);
    for i=1:N, nxt=i+1; if nxt>N, nxt=1; end; f(i,:) = [1, i+1, nxt+1]; end
end

function v_out = transform_verts(v_in, R, offset_vec, map_func)
    N = size(v_in, 1); v_out = zeros(N, 3);
    for i=1:N, pt_ned = R * v_in(i,:)'; pt_plot = map_func(pt_ned); v_out(i,:) = (offset_vec + pt_plot)'; end
end

function R = get_rotation_matrix(euler)
    phi = euler(1); theta = euler(2); psi = euler(3);
    c_ph = cos(phi); s_ph = sin(phi); c_th = cos(theta); s_th = sin(theta); c_ps = cos(psi); s_ps = sin(psi);
    R = [ c_th*c_ps,   s_ph*s_th*c_ps - c_ph*s_ps,   c_ph*s_th*c_ps + s_ph*s_ps;
          c_th*s_ps,   s_ph*s_th*s_ps + c_ph*c_ps,   c_ph*s_th*s_ps - s_ph*c_ps;
         -s_th,        s_ph*c_th,                    c_ph*c_th                 ];
end