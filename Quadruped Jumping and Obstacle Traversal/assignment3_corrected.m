% LIPM_Simulation.m
% Reproduces Figure 2(a) and 2(b) for the Linear Inverted Pendulum Mode

% --- System Parameters ---
g = 981.0;      % Gravity [cm/s^2] (using CGS to match paper units)
m = 2.0;        % Mass [kg]
h = 8.0;        % Distance between hip joint and center of mass [cm]

% --- Initial Conditions ---
x0 = -7.0;      % Initial x position [cm]
dx0 = 41.0;     % Initial x velocity [cm/s] (Note: paper typo says [cm])

% --- Simulation Time ---
% 16 discrete states are shown in the original plots. 
% A time horizon of 0.8 seconds perfectly captures the x = -7 to x = +8 sweep.
t_end = 0.8;    
N = 16;         
t = linspace(0, t_end, N); 

% Setup Figure window
figure('Color', 'white', 'Position', [100, 100, 700, 900]);
%    Position[left edge, bottom edge, width, height]
%tall window to hold 2 subplots

%% ================= Figure 2(a): Slope Trajectory =================
subplot(2, 1, 1);
hold on;
k_a = 0.3;
yH_a = 30.0;
omega_c_a = 0.0; % Constant vertical torso

% Dynamics
omega_a = sqrt(g / yH_a); %Computes the LIPM natural frequency
%omega = \sqrt{\frac{g}{y_H}}
x_a = x0 * cosh(omega_a * t) + (dx0 / omega_a) * sinh(omega_a * t);
%Assume a solution of the form
%x=e^{rt}
y_a = k_a * x_a + yH_a; % Slope trajectory constraint
theta_a = zeros(1, N);  % Torso remains vertical

% Plot dashed trajectory line
plot([-15, 12], k_a * [-15, 12] + yH_a, 'k:', 'LineWidth', 1.2);
%'k:'
%means:
%* k = black
%* : = dotted

% Draw the pendulum at each timestep
for i = 1:N
    % Leg: from foot (0,0) to hip (x,y)
    plot([0, x_a(i)], [0, y_a(i)], 'k-', 'LineWidth', 0.8);
    % Hip Joint
    plot(x_a(i), y_a(i), 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'w');
    % Torso: from hip to Center of Mass
    x_top = x_a(i) + h * sin(theta_a(i));
    y_top = y_a(i) + h * cos(theta_a(i));
    plot([x_a(i), x_top], [y_a(i), y_top], 'k-', 'LineWidth', 0.8);
end

% Formatting for 2(a)
axis equal; axis([-12 12 -15 45]); %first 2 x1 to x2, next 2 y1 to y2
%without this robot appeared stretched 
set(gca, 'Visible', 'off'); % Hide default axes to draw custom ones

% Custom X-axis with downward ticks [1cm/div]
plot([-12, 12], [0, 0], 'k-', 'LineWidth', 1);
for xt = -12:12
    plot([xt, xt], [0, -0.8], 'k-', 'LineWidth', 0.8);
end
text(2, -2.5, '[1cm/div]', 'FontName', 'Times New Roman', 'FontSize', 11, 'Color', 'k');


%% ======== Figure 2(b): Horizontal trajectory, rotating body ========
subplot(2, 1, 2);
hold on;
k_b = 0.0;
yH_b = 30.0;
omega_c_b = 1.0; 

% Dynamics (Horizontal decoupled motion is identical to sloped)
omega_b = sqrt(g / yH_b);
x_com = x0 * cosh(omega_b * t) + (dx0 / omega_b) * sinh(omega_b * t);


% Torso rotates. Initial angle estimated at -0.4 rad for symmetry
theta_0 = -0.4; 
theta_b = theta_0 + omega_c_b * t;

% The horizontal trajectory constraint applies to the CoM (y = 30), not the hip.
% Therefore, the hip must dip down and shift horizontally relative to the CoM.
x_b = x_com - h * sin(theta_b);
y_b = yH_b - h * cos(theta_b);

% Plot dashed trajectory line
plot([-15, 12], [yH_b, yH_b], 'k:', 'LineWidth', 1.2);

% Draw the pendulum at each timestep
for i = 1:N
    % Leg: from foot (0,0) to hip (x,y)
    plot([0, x_b(i)], [0, y_b(i)], 'k-', 'LineWidth', 0.8);
    % Hip Joint
    plot(x_b(i), y_b(i), 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'w');
    % Torso: from hip to Center of Mass
    x_top = x_b(i) + h * sin(theta_b(i));
    y_top = y_b(i) + h * cos(theta_b(i));
    plot([x_b(i), x_top], [y_b(i), y_top], 'k-', 'LineWidth', 0.8);
end

% Formatting for 2(b)
axis equal; axis([-12 12 -30 45]);
set(gca, 'Visible', 'off');

% Custom X-axis with downward ticks [1cm/div]
plot([-12, 12], [0, 0], 'k-', 'LineWidth', 1);
for xt = -12:12
    plot([xt, xt], [0, -0.8], 'k-', 'LineWidth', 0.8);
end
text(2, -2.5, '[1cm/div]', 'FontName', 'Times New Roman', 'FontSize', 11, 'Color', 'k');
