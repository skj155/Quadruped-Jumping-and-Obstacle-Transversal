% LIPM_Stair_Climb.m
% Simulates a biped climbing stairs using Orbital Energy leg exchange
clc; clear; close all;

%% 1. Parameter Definitions
g = 9.81;           % Acceleration due to gravity [m/s^2]
yH = 0.8;           % Constant COM height constraint [m]
s = 0.4;            % Stride/Step length [m]
h_step = 0.05;      % Height of each stair step [m] (The 'h' parameter)
num_steps = 5;      % Number of steps to simulate

%% 2. Orbital Energy Leg Exchange Calculation
% Using Eq. 18 : E2 - E1 = g * h_step
% This calculates the shifted exchange point (xf) to build enough energy
xf = (s / 2) + (yH * h_step) / s;

fprintf('Nominal flat exchange point: %.3f m\n', s/2);
fprintf('Dynamic stair exchange point (xf): %.3f m\n', xf);

%% 3. Initial Conditions
x0 = -s/2;          % Initial relative COM position [m]
v0 = 1.5;           % Initial COM velocity [m/s] (Needs more speed to climb!)

% Tracking absolute positions in the global frame
current_foot_X = 0; 
current_foot_Y = 0; 
t_start = 0;        

%% 4. Data Logging Arrays
T_log = [];         
X_abs_log = [];     
Y_abs_log = [];
V_log = [];         
Foot_X_log = [];  
Foot_Y_log = [];

%% 5. Main Simulation Loop
for step = 1:num_steps
    
    % Set ODE options to trigger event precisely at our newly calculated xf
    options = odeset('Events', @(t, state) support_exchange_event(t, state, xf));
    
    % Simulate continuous dynamics for the current step
    [t, state] = ode45(@(t, state) lipm_dynamics(t, state, g, yH), ...
                       [t_start, t_start + 2], [x0, v0], options);
    
    x_rel = state(:, 1);
    v_rel = state(:, 2);
    
    % Convert to absolute global positions
    x_abs = x_rel + current_foot_X;
    y_abs = current_foot_Y + yH; % COM is always yH above the current foot
    
    % Log data 
    T_log = [T_log; t];
    X_abs_log = [X_abs_log; x_abs];
    Y_abs_log = [Y_abs_log; y_abs * ones(size(t))];
    V_log = [V_log; v_rel];
    Foot_X_log = [Foot_X_log; current_foot_X * ones(size(t))];
    Foot_Y_log = [Foot_Y_log; current_foot_Y * ones(size(t))];
    
    %% --- Coordinate Transformation (Stair Leg Exchange) ---
    % 1. Foot moves forward by stride length (s) and UP by step height (h_step)
    current_foot_X = current_foot_X + s;
    current_foot_Y = current_foot_Y + h_step;
    
    % 2. Shift relative COM position backwards
    x0 = x_rel(end) - s; 
    
    % 3. Velocity is conserved (e = 1)
    v0 = v_rel(end);
    
    t_start = t(end);
end

%% 6. Plotting Results
figure('Name', 'LIPM Stair Climbing Simulation', 'Position', [100, 100, 1000, 800]);

% Plot 1: Absolute X Position
subplot(3,1,1);
plot(T_log, X_abs_log, 'b-', 'LineWidth', 1.5); hold on;
plot(T_log, Foot_X_log, 'r--', 'LineWidth', 1.5);
title('Absolute Horizontal Position vs Time');
xlabel('Time [s]'); ylabel('Position X [m]');
legend('COM X Position', 'Foot X Position', 'Location', 'northwest');
grid on;

% Plot 2: Absolute Y Position (Showing the stepping up motion)
subplot(3,1,2);
plot(T_log, Y_abs_log, 'b-', 'LineWidth', 1.5); hold on;
plot(T_log, Foot_Y_log, 'r--', 'LineWidth', 1.5);
title('Absolute Vertical Position vs Time (Stair Climbing)');
xlabel('Time [s]'); ylabel('Height Y [m]');
legend('COM Height', 'Ground Height', 'Location', 'northwest');
grid on;

% Plot 3: COM Velocity
subplot(3,1,3);
plot(T_log, V_log, 'y-', 'LineWidth', 1.5);
title('COM Velocity vs Time');
xlabel('Time [s]'); ylabel('Velocity [m/s]');
grid on;

%% 7. Animation: Stair Climbing
figure('Name', 'LIPM Stair Climbing Animation', 'Position', [150, 150, 900, 400]);
hold on; grid minor; axis equal;

xlim([-s, max(X_abs_log) + s]);
ylim([-0.2, max(Y_abs_log) + 0.4]);

% Draw the rugged terrain (staircase)
for i = 1:num_steps+1
    x_start = (i-2)*s;
    x_end = x_start + s;
    y_level = (i-1)*h_step;
    
    % Draw flat step
    plot([x_start, x_end], [y_level, y_level], 'g-', 'LineWidth', 2);
    % Draw vertical riser to next step
    if i <= num_steps
        plot([x_end, x_end], [y_level, y_level + h_step], 'g-', 'LineWidth', 2);
    end
end

title(['LIPM: Stair Climbing (h = ' num2str(h_step) 'm)']);
xlabel('Global Position [m]'); ylabel('Height [m]');

% Initialize animation objects
leg_line = plot([Foot_X_log(1), X_abs_log(1)], [Foot_Y_log(1), Y_abs_log(1)], 'b-', 'LineWidth', 2);
com_body = plot(X_abs_log(1), Y_abs_log(1), 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 12);
foot_contact = plot(Foot_X_log(1), Foot_Y_log(1), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8);

% Animation Loop
for i = 1:1:length(T_log)
    set(leg_line, 'XData', [Foot_X_log(i), X_abs_log(i)], 'YData', [Foot_Y_log(i), Y_abs_log(i)]);
    set(com_body, 'XData', X_abs_log(i), 'YData', Y_abs_log(i));
    set(foot_contact, 'XData', Foot_X_log(i), 'YData', Foot_Y_log(i));
    
    drawnow;
    
    if i < length(T_log)
        pause(T_log(i+1) - T_log(i)); 
    end
end

%% Helper Functions 
function dxdt = lipm_dynamics(~, state, g, yH)
    dxdt = zeros(2,1);
    dxdt(1) = state(2);                  
    dxdt(2) = (g / yH) * state(1);       
end

% xf is now passed in dynamically instead of hardcoded to s/2!
function [value, isterminal, direction] = support_exchange_event(~, state, xf)
    x = state(1);
    value = x - xf;  % Trigger event exactly at calculated xf
    isterminal = 1;     
    direction = 1;      
end