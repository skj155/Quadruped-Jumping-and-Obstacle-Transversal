% =========================================================================
% Human-Like Predictive Locomotion: 5-Link Planar Biped
% =========================================================================

clear; clc; close all;

%% 1. SYSTEM PARAMETERS & INITIALIZATION
g = 9.81;          
y_H = 0.5;         
Tc = sqrt(y_H/g);  

L_shin = 0.4;
L_thigh = 0.4;
L_torso = 0.3;

% --- THE FIX: Alternating Footstep Sequence ---
% Defining the exact footprint locations so the legs step PAST each other
footsteps = [
    -0.30,  0.00;  % Initial Back Foot (Leg 2 starting position)
     0.00,  0.00;  % Initial Front Foot (Leg 1 anchor point)
     0.30,  0.05;  % Step 1: Leg 2 swings here
     0.60,  0.02;  % Step 2: Leg 1 swings here
     0.90,  0.10;  % Step 3: Leg 2 swings here (Stair)
     1.20,  0.10;  % Step 4: Leg 1 swings here
     1.50,  0.00   % Step 5: Leg 2 swings here (Down)
];

% A step requires a start, an anchor, and an end, so we subtract 2
num_steps = size(footsteps, 1) - 2; 

%% 2. SIMULATION SETUP
dt = 0.02;         
T_ss = 0.6;        
time_steps = 0:dt:T_ss;
n_frames = length(time_steps);

% Arrays to store global animation data mapped to specific legs
history_L1_foot = []; history_L1_knee = []; 
history_L2_foot = []; history_L2_knee = []; 
history_hip = []; history_torso = [];

disp('Generating Alternating Gait Trajectories...');

%% 3. MAIN WALKING LOOP
for step_idx = 1:num_steps
    
    % Step-Through Logic: Identify where feet start and end
    sw_start = footsteps(step_idx, :);     % The trailing foot in the back
    st_foot  = footsteps(step_idx + 1, :); % The anchor foot in the middle
    sw_end   = footsteps(step_idx + 2, :); % The target foot in the front
    
    step_length = sw_end(1) - st_foot(1);
    
    % Symmetric LIPM Boundary Conditions for stability
    CoM_x0 = -step_length / 2; 
    CoM_v0 = (step_length / 2) * (1 + cosh(T_ss/Tc)) / (Tc * sinh(T_ss/Tc));
    
    % Temporary arrays for this specific step
    step_L1_foot = zeros(n_frames, 2); step_L1_knee = zeros(n_frames, 2);
    step_L2_foot = zeros(n_frames, 2); step_L2_knee = zeros(n_frames, 2);
    step_hip =     zeros(n_frames, 2); step_torso =   zeros(n_frames, 2);
    
    for k = 1:n_frames
        t = time_steps(k);
        s = t / T_ss; % Normalized phase (0 to 1)
        
        % 1. LIPM CoM Trajectory
        CoM_x = CoM_x0 * cosh(t/Tc) + CoM_v0 * Tc * sinh(t/Tc);
        CoM_z = st_foot(2) + y_H;
        
        Hip_x = st_foot(1) + CoM_x;
        Hip_z = CoM_z;
        
        % 2. Quintic Polynomial Swing Foot Trajectory (From back to front)
        h_max = max(sw_start(2), sw_end(2)) + 0.15; % 15cm clearance
        
        sw_x = sw_start(1) + (sw_end(1) - sw_start(1)) * (10*s^3 - 15*s^4 + 6*s^5);
        
        if s <= 0.5
            sr = s / 0.5;
            sw_z = sw_start(2) + (h_max - sw_start(2)) * (10*sr^3 - 15*sr^4 + 6*sr^5);
        else
            sf = (s - 0.5) / 0.5;
            sw_z = h_max + (sw_end(2) - h_max) * (10*sf^3 - 15*sf^4 + 6*sf^5);
        end
        
        % 3. Exact Inverse Kinematics
        st_knee = solve_2link_ik(st_foot, [Hip_x, Hip_z], L_shin, L_thigh, 'backward');
        sw_knee = solve_2link_ik([Hip_x, Hip_z], [sw_x, sw_z], L_thigh, L_shin, 'forward');
        
        Torso_x = Hip_x;
        Torso_z = Hip_z + L_torso;
        
        % --- ROLE ASSIGNMENT: Alternating Legs ---
        if mod(step_idx, 2) == 1 
            % Odd Step: Leg 1 (Blue) is Stance, Leg 2 (Red) is Swing
            step_L1_foot(k,:) = st_foot;
            step_L1_knee(k,:) = st_knee;
            step_L2_foot(k,:) = [sw_x, sw_z];
            step_L2_knee(k,:) = sw_knee;
        else
            % Even Step: Leg 2 (Red) is Stance, Leg 1 (Blue) is Swing
            step_L2_foot(k,:) = st_foot;
            step_L2_knee(k,:) = st_knee;
            step_L1_foot(k,:) = [sw_x, sw_z];
            step_L1_knee(k,:) = sw_knee;
        end
        
        step_hip(k,:) = [Hip_x, Hip_z];
        step_torso(k,:) = [Torso_x, Torso_z];
    end
    
    % Append to global history
    history_L1_foot = [history_L1_foot; step_L1_foot];
    history_L1_knee = [history_L1_knee; step_L1_knee];
    history_L2_foot = [history_L2_foot; step_L2_foot];
    history_L2_knee = [history_L2_knee; step_L2_knee];
    history_hip =     [history_hip; step_hip];
    history_torso =   [history_torso; step_torso];
    
end

%% 4. VISUALIZATION ENGINE
disp('Rendering Human-Like Walk Animation...');
figure('Name', 'Alternating 5-Link Walk', 'Position', [100, 100, 1000, 500]);
hold on; grid on; axis equal;
xlim([-0.5, footsteps(end,1) + 0.3]);
ylim([-0.1, y_H + L_torso + 0.2]);
xlabel('Horizontal Position (m)');
ylabel('Vertical Position (m)');
title('Human-Like Alternating Gait: 5-Link Biped');

ground_color = [1.0, 1.0, 0.0];

for i = 1:size(history_hip, 1)
    cla;
    
    % Draw Terrain Outline
    plot(footsteps(:,1), footsteps(:,2), 'k--', 'LineWidth', 1);
    for j = 1:size(footsteps,1)-1
        plot([footsteps(j,1), footsteps(j+1,1)], [footsteps(j,2), footsteps(j+1,2)], 'k-', 'LineWidth', 2);
    end
    
    % Extract joints
    L1_f = history_L1_foot(i,:); L1_k = history_L1_knee(i,:); 
    L2_f = history_L2_foot(i,:); L2_k = history_L2_knee(i,:); 
    hip = history_hip(i,:); tor = history_torso(i,:);
    
    % Draw Leg 1 (ALWAYS Blue)
    plot([L1_f(1), L1_k(1)], [L1_f(2), L1_k(2)], 'b-', 'LineWidth', 4);
    plot([L1_k(1), hip(1)],  [L1_k(2), hip(2)],  'b-', 'LineWidth', 4);
    
    % Draw Leg 2 (ALWAYS Red)
    plot([L2_f(1), L2_k(1)], [L2_f(2), L2_k(2)], 'r-', 'LineWidth', 4);
    plot([L2_k(1), hip(1)],  [L2_k(2), hip(2)],  'r-', 'LineWidth', 4);
    
    % Draw Torso (Black)
    plot([hip(1), tor(1)],   [hip(2), tor(2)],   'k-', 'LineWidth', 5);
    
    % Draw Joints
    plot([L1_f(1), L1_k(1), L2_f(1), L2_k(1), hip(1), tor(1)], ...
         [L1_f(2), L1_k(2), L2_f(2), L2_k(2), hip(2), tor(2)], ...
         'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 6);
         
    drawnow;
end
disp('Simulation Finished.');

%% --- HELPER FUNCTION: GEOMETRIC INVERSE KINEMATICS ---
function joint_pos = solve_2link_ik(p_base, p_end, L1, L2, bend_dir)
    Dx = p_end(1) - p_base(1);
    Dz = p_end(2) - p_base(2);
    D = sqrt(Dx^2 + Dz^2);
    
    if D > (L1 + L2)
        D = L1 + L2 - 1e-4; 
    end
    
    gamma = acos((L1^2 + D^2 - L2^2) / (2 * L1 * D));
    alpha = atan2(Dz, Dx);
    
    % This keeps the knees pointing forward like a human
    if strcmp(bend_dir, 'backward')
        theta1 = alpha - gamma;
    else
        theta1 = alpha + gamma;
    end
    
    joint_pos = [p_base(1) + L1 * cos(theta1), p_base(2) + L1 * sin(theta1)];
end