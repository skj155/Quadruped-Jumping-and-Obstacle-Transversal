function lipm_simulation()
    % Parameters from Section 2 & Figure 2 of the paper
    g = 981;          % Gravity acceleration (cm/s^2) - converted to cm for scaling
    m = 2.0;          % Body mass (kg) [cite: 119]
    y_H = 30.0;       % Intersection with y-axis (cm) [cite: 103, 118]
    
    % Initial conditions for both cases [cite: 103, 118]
    x0 = -7.0;        % Initial position (cm)
    dx0 = 41.0;       % Initial velocity (cm/s)
    
    % Time span for simulation
    t_span = 0:0.01:0.5; % 0.5 seconds duration
    
    % --- CASE (a): Slope Trajectory ---
    k_a = 0.3;         % Slope of constraint line [cite: 103]
    omega_a = 0.0;     % Constant angular velocity (rad/s) [cite: 103]
    [x_a, y_a, theta_a] = simulate_lipm(t_span, g, y_H, x0, dx0, k_a, omega_a);
    
    % --- CASE (b): Horizontal Trajectory, Rotating Body ---
    k_b = 0.0;         % Slope of constraint line 
    omega_b = 1.0;     % Constant angular velocity (rad/s) 
    [x_b, y_b, theta_b] = simulate_lipm(t_span, g, y_H, x0, dx0, k_b, omega_b);
    
    
    % PLOTTING & ANIMATION
   
    figure('Name', 'Linear Inverted Pendulum Mode Simulation', 'Position', [100, 100, 1000, 500]);
    
    % Subplot 1: Slope Trajectory
    subplot(1, 2, 1);
    hold on; grid on; axis equal;
    xlim([-15, 20]); ylim([-5, 45]);
    title('Slope Trajectory (k = 0.3)');
    xlabel('x [cm]'); ylabel('y [cm]');
    
    % Subplot 2: Horizontal Trajectory
    subplot(1, 2, 2);
    hold on; grid on; axis equal;
    xlim([-15, 20]); ylim([-5, 45]);
    title('Horizontal Trajectory, Rotating Body (\omega_c = 1.0)');
    xlabel('x [cm]'); ylabel('y [cm]');
    
    % Plot static elements (Ground and Ankle Joint at Origin O)
    plot(subplot(1,2,1), [-15, 20], [0, 0], 'k', 'LineWidth', 2);
    plot(subplot(1,2,1), 0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
    plot(subplot(1,2,2), [-15, 20], [0, 0], 'k', 'LineWidth', 2);
    plot(subplot(1,2,2), 0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
    
    % Plot the constraint lines: y = k*x + y_H [cite: 80]
    x_range = -15:20;
    plot(subplot(1,2,1), x_range, k_a*x_range + y_H, 'r--', 'LineWidth', 1);
    plot(subplot(1,2,2), x_range, k_b*x_range + y_H, 'r--', 'LineWidth', 1);
    
    % Initialize animated graphics objects
    link_a = plot(subplot(1,2,1), [0, x_a(1)], [0, y_a(1)], 'b-', 'LineWidth', 2);
    body_a = plot_ellipse(subplot(1,2,1), x_a(1), y_a(1), theta_a(1));
    
    link_b = plot(subplot(1,2,2), [0, x_b(1)], [0, y_b(1)], 'b-', 'LineWidth', 2);
    body_b = plot_ellipse(subplot(1,2,2), x_b(1), y_b(1), theta_b(1));
    
    % Trace paths
    plot(subplot(1,2,1), x_a, y_a, 'g:', 'LineWidth', 1.5);
    plot(subplot(1,2,2), x_b, y_b, 'g:', 'LineWidth', 1.5);
    
    % Animation Loop
    for i = 1:length(t_span)
        % Update Case A
        set(link_a, 'XData', [0, x_a(i)], 'YData', [0, y_a(i)]);
        update_ellipse(body_a, x_a(i), y_a(i), theta_a(i));
        
        % Update Case B
        set(link_b, 'XData', [0, x_b(i)], 'YData', [0, y_b(i)]);
        update_ellipse(body_b, x_b(i), y_b(i), theta_b(i));
        
        drawnow;
        pause(0.02); % Control animation speed
    end
end



function [x, y, theta] = simulate_lipm(t, g, y_H, x0, dx0, k, omega_c)
    % Analytical solution of d^2(x)/dt^2 = (g/y_H) * x 
    T_c = sqrt(y_H / g); 
    
    % x(t) and dx(t) solutions using hyperbolic functions
    x = x0 * cosh(t / T_c) + dx0 * T_c * sinh(t / T_c);
    
    % Calculate y based on the linear constraint line equation (8) 
    y = k * x + y_H;
    
    % Calculate body attitude theta based on equation (9) 
    theta = omega_c * t; 
end

function h = plot_ellipse(ax, cx, cy, angle)
    % Plots an oval/ellipse representing the robot body 
    t = linspace(0, 2*pi, 50);
    rx = 2.5; % Horizontal radius (cm)
    ry = 5.0; % Vertical radius (cm)
    
    % Base ellipse at origin
    xe = rx * cos(t);
    ye = ry * sin(t);
    
    % Rotate and translate
    R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    coords = R * [xe; ye];
    
    h = plot(ax, coords(1,:) + cx, coords(2,:) + cy, 'k-', 'LineWidth', 2);
    % Fill color to represent the Center of Mass pattern
    fill(ax, coords(1,:) + cx, coords(2,:) + cy, [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
end

function update_ellipse(h, cx, cy, angle)
    % Updates the orientation and position of the body ellipse during animation
    t = linspace(0, 2*pi, 50);
    rx = 2.5; ry = 5.0;
    xe = rx * cos(t); ye = ry * sin(t);
    R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    coords = R * [xe; ye];
    set(h, 'XData', coords(1,:) + cx, 'YData', coords(2,:) + cy);
end