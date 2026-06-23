%hori smooth terrain
clc; clear; close all;

%% 1. Parameter Definitions
g = 9.81;           % Acceleration due to gravity [m/s^2]
yH = 0.8;           % Constant COM height constraint [m]
s = 0.4;            % Stride/Step length [m]
num_steps = 5;      % Number of steps to simulate

%% 2. Initial Conditions
% We start at the beginning of a step, with the COM behind the support foot.
x0 = -s/2;          % Initial relative COM position [m]
v0 = 1.2;           % Initial COM velocity [m/s] (Must be high enough to push COM past x=0)

% Tracking the absolute global position of the support foot
current_foot_pos = 0; 
t_start = 0;        % Global time tracker

%% 3. Data Logging Arrays
T_log = [];         % Time [s]
X_abs_log = [];     % Absolute global COM position [m]
V_log = [];         % COM velocity [m/s]
Foot_pos_log = [];  % Absolute foot position [m]

%% 4. Main Simulation Loop
for step = 1:num_steps
    
    % Set ODE options to trigger 'support_exchange_event'
    % This stops the solver exactly when x reaches s/2
    options = odeset('Events', @(t, state) support_exchange_event(t, state, s));
    
    % Simulate continuous dynamics for the current step
    % We give it a generous time window [t_start, t_start + 2 seconds]
    [t, state] = ode45(@(t, state) lipm_dynamics(t, state, g, yH), ...
                       [t_start, t_start + 2], [x0, v0], options);
    
    % Extract relative state variables
    x_rel = state(:, 1);
    v_rel = state(:, 2);
    
    % Convert relative position to absolute global position
    x_abs = x_rel + current_foot_pos;
    
    %These lines take the master lists (T_log, X_abs_log, etc.) and attach the newly calculated data (t, x_abs, v_rel) to the bottom of them. The semicolon ; ensures they stack vertically.
    % Log data for plotting
    T_log = [T_log; t];
    X_abs_log = [X_abs_log; x_abs];
    V_log = [V_log; v_rel];
    Foot_pos_log = [Foot_pos_log; current_foot_pos * ones(size(t))];
    
    %% --- Coordinate Transformation (Support Leg Exchange) ---
    % 1. The foot position moves forward by one stride length 's'
    % (Modifying this line later allows for uneven terrain step lengths)
    current_foot_pos = current_foot_pos + s;
    
    % 2. Shift the relative COM position backwards by 's'
    x0 = x_rel(end) - s; 
    
    % 3. Velocity is conserved (e = 1)
    v0 = v_rel(end);
    
    % Update global time for the next ODE iteration
    t_start = t(end);
end

%% 5. Plotting Results
figure('Name', 'LIPM Biped Simulation', 'Position', [100, 100, 1000, 800]);

% Plot 1: Absolute COM Position vs Time
subplot(3,1,1);
plot(T_log, X_abs_log, 'b-', 'LineWidth', 1.5); hold on;
plot(T_log, Foot_pos_log, 'r--', 'LineWidth', 1.5);
title('Absolute COM Position & Foot Placements vs Time');
xlabel('Time [s]'); ylabel('Position [m]');
legend('COM Position', 'Support Foot Position', 'Location', 'northwest');
grid on;
%because feet change only at step transitions so moving in jumps

% Plot 2: COM Velocity vs Time
subplot(3,1,2);
plot(T_log, V_log, 'k-', 'LineWidth', 1.5);
title('COM Velocity vs Time');
xlabel('Time [s]'); ylabel('Velocity [m/s]');
grid on;
%Since LIPM behaves like a falling pendulum:
%velocity increases as COM moves forward

% Plot 3: Phase Portrait (Relative x vs. v)
subplot(3,1,3);
% Recalculate relative x for the phase plot to show the repetitive loops
X_rel_log = X_abs_log - Foot_pos_log;
plot(X_rel_log, V_log, 'm-', 'LineWidth', 1.5); hold on;
plot(X_rel_log(1), V_log(1), 'go', 'MarkerFaceColor', 'g'); % Start point
plot(X_rel_log(end), V_log(end), 'ro', 'MarkerFaceColor', 'r'); % End point
title('Phase Portrait (Relative Position vs Velocity)');
xlabel('Relative Position x [m]'); ylabel('Velocity \dot{x} [m/s]');
grid on;


%% --- Helper Functions ---

% Dynamics function for the LIPM
function dxdt = lipm_dynamics(~, state, g, yH)
    % state(1) = x, state(2) = dx/dt
    dxdt = zeros(2,1);
    dxdt(1) = state(2);                  % dx/dt = velocity
    dxdt(2) = (g / yH) * state(1);       % d^2x/dt^2 = (g/yH) * x
end

% Event function to detect support leg exchange
function [value, isterminal, direction] = support_exchange_event(~, state, s)
    x = state(1);
    % We want to trigger the event when x reaches s/2
    value = x - (s/2);  
    
    % Stop the integration when the event occurs
    isterminal = 1;     
    
    % We only care when x is increasing (moving forward)
    direction = 1;      
end

%% 6. Animation: Support Leg Exchange on Horizontal Surface
% Run this immediately after your main simulation and plotting blocks

figure('Name', 'LIPM Biped Animation', 'Position', [150, 150, 900, 400]);
hold on; grid minor; axis equal;

% Set viewing window based on the total distance traveled
xlim([-s, max(X_abs_log) + s]);
ylim([-0.2, yH + 0.4]);

% Draw the horizontal ground surface
plot([-s, max(X_abs_log) + s], [0, 0], 'k-', 'LineWidth', 2);

title('LIPM: Support Leg Exchange (Horizontal Surface)');
xlabel('Global Position [m]'); ylabel('Height [m]');

% Initialize the graphics objects for the animation
% 1. The rigid leg (line from foot to COM)
leg_line = plot([Foot_pos_log(1), X_abs_log(1)], [0, yH], 'b-', 'LineWidth', 2);

% 2. The torso / Center of Mass (represented as a distinct circle)
com_body = plot(X_abs_log(1), yH, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 12);

% 3. The current foot placement on the ground
foot_contact = plot(Foot_pos_log(1), 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8);

% 4. Trail of previous foot placements to show the stride history
foot_trail = plot(Foot_pos_log(1), 0, 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);

% --- Animation Loop ---
for i = 1:length(T_log)
    curr_x = X_abs_log(i);
    curr_foot = Foot_pos_log(i);

    % Update the visual coordinates dynamically
    set(leg_line, 'XData', [curr_foot, curr_x], 'YData', [0, yH]);
    set(com_body, 'XData', curr_x, 'YData', yH);
    set(foot_contact, 'XData', curr_foot, 'YData', 0);

    % Update the foot trail (only unique foot locations)
    past_feet = unique(Foot_pos_log(1:i));
    set(foot_trail, 'XData', past_feet, 'YData', zeros(size(past_feet)));

    % Force MATLAB to render the current frame
    drawnow;

    % Match playback speed to the simulated time delta
    if i < length(T_log)
        dt = T_log(i+1) - T_log(i);
        pause(dt); 
    end
end