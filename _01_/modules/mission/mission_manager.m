function target = mission_manager(t)
% File Name: mission_manager.m
% Position: Root > modules > mission > mission_manager.m
% Description: Defines flight phases and waypoints.

    % Default
    target.pos   = [0; 0; 0];
    target.euler = [0; 0; 0];
    
    % Phase 1: Takeoff (0 - 5s) -> Altitude 2m
    if t < 5.0
        target.pos   = [0; 0; -2.0];
        target.euler = deg2rad([20; 0; 0]);
        
    % Phase 2: Move Forward (5 - 10s) -> North 5m
    elseif t < 10.0
        target.pos   = [30; 30; -2.0];
        target.euler = deg2rad([0; -30; 0]);
        
    % Phase 3: Move Sideways & Yaw (10 - 15s) -> East 2m, Yaw 45 deg
    elseif t < 15.0
        target.pos   = [-10; 60; -2.0];
        target.euler = deg2rad([0; 0; -100]);
        
    % Phase 4: Hold
    else
        target.pos   = [0; 0; -10.0];
        target.euler = deg2rad([0; 0; 50]);
    end
end