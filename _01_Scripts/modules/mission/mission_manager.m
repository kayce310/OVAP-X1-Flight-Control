function target = mission_manager(t)
% File Name: mission_manager.m
% Position: Root > modules > mission > mission_manager.m
% Description: Defines flight phases and waypoints.

    % Default
    target.pos   = [0; 0; 0];
    target.euler = [0; 0; 0];
    
    if t < 5.0
        target.pos   = [0; 0; -5.0];
        target.euler = deg2rad([0; 0; 0]);

    elseif t < 10.0
        target.pos   = [10; 0; -5.0];
        target.euler = deg2rad([0; 0; 0]);

    elseif t < 15.0
        target.pos   = [0; 10; -10.0];
        target.euler = deg2rad([0; 0; 45]);

    elseif t < 30.0
        target.pos   = [10; 10; -10.0];
        target.euler = deg2rad([0; 0; 45]);

    % 
    else
        target.pos   = [0; 0; -10.0];
        target.euler = deg2rad([0; 70; 0]);
    end
end