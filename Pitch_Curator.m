%% AUTONOMOUS PITCH CURATOR (FINAL ROBUST)
% -------------------------------------------------------------------------
% Objective: Randomized Defect Detection & Repair on Standard Cricket Pitch.
% Optimization: Path spacing reduced to 6 units (25% overlap) to prevent 
%               missed detections of small random defects.
% -------------------------------------------------------------------------

clear; clc; close all;

%% 1. SETUP ENVIRONMENT & RANDOM DEFECTS
fprintf('Initializing Randomized Simulation...\n');

% A. Standard Dimensions (20.12m x 3.05m)
real_L_cm = 2012; 
real_W_cm = 305;  
scale = 5; % 1 unit = 5 cm
pitch_L = round(real_L_cm / scale); 
pitch_W = round(real_W_cm / scale);

% B. Energy Constants (The "Cost" Model)
energy_move = 1.5;    % Joules per step
energy_repair = 15.0; % Joules per tamper actuation

% C. Generate Base Ground (Soil Noise)
ground_truth_initial = 0.05 * randn(pitch_W, pitch_L); 

% D. Generate RANDOM Defect Clusters
num_defects = randi([5, 8]); % Generate 5 to 8 random spots
fprintf('Generating %d random defect patches...\n', num_defects);

defect_mask = zeros(pitch_W, pitch_L);

for k = 1:num_defects
    % Random Size (approx 30cm to 75cm)
    w_defect = randi([6, 15]); 
    h_defect = randi([6, 15]);
    
    % Random Position (Safe bounds)
    x_pos = randi([5, pitch_L - w_defect - 5]); 
    y_pos = randi([5, pitch_W - h_defect - 5]);
    
    % Apply to mask
    defect_mask(y_pos:y_pos+h_defect, x_pos:x_pos+w_defect) = 1;
end

% Apply Defects (Depth: -2.0mm to -4.0mm)
defect_depths = defect_mask .* (2.0 + (2.0 * rand(pitch_W, pitch_L)));
ground_truth_initial = ground_truth_initial - defect_depths; 
ground_map = ground_truth_initial; % Working copy

%% 2. PATH PLANNING (OPTIMIZED FOR OVERLAP)
waypoints = [];
robot_width = 6; % REDUCED from 8 to 6 to guarantee overlap (Safety Factor)
margin = 5;

% Generate Zig-Zag Path
for y = margin:robot_width:pitch_W-margin
    if mod((y-margin)/robot_width, 2) == 0
        waypoints = [waypoints; margin, y; pitch_L-margin, y]; % Left -> Right
    else
        waypoints = [waypoints; pitch_L-margin, y; margin, y]; % Right -> Left
    end
end

% Interpolate Path (High Resolution)
path = [];
for i=1:size(waypoints,1)-1
    p1 = waypoints(i,:); p2 = waypoints(i+1,:);
    dist = norm(p2-p1);
    num_steps = ceil(dist); 
    path = [path; linspace(p1(1), p2(1), num_steps)', linspace(p1(2), p2(2), num_steps)'];
end

%% 3. EXECUTE MISSION
fprintf('Simulating %d Steps... ', length(path));

log_time = 1:length(path);
log_Ra = zeros(1, length(path));   
log_Rrms = zeros(1, length(path)); 
log_energy = zeros(1, length(path));
log_coverage = zeros(1, length(path));

covered_mask = zeros(pitch_W, pitch_L);
current_energy = 0;
repair_count = 0;

for t = 1:length(path)
    rx = round(path(t,1)); 
    ry = round(path(t,2));
    
    % Safety Bounds
    rx = max(6, min(pitch_L-6, rx));
    ry = max(6, min(pitch_W-6, ry));
    
    % A. Update Coverage Map
    r_idx = max(1, ry-5):min(pitch_W, ry+5);
    c_idx = max(1, rx-5):min(pitch_L, rx+5);
    covered_mask(r_idx, c_idx) = 1;
    log_coverage(t) = (sum(covered_mask(:)) / numel(covered_mask)) * 100;
    
    % B. Smart Repair Logic
    patch = ground_map(r_idx, c_idx);
    step_cost = energy_move; % Base movement cost
    
    % Threshold: If any point is deeper than -1.0mm
    if min(patch(:)) < -1.0 
        % ACTUATE TAMPER: Flatten area with noise
        ground_map(r_idx, c_idx) = 0.05 * randn(length(r_idx), length(c_idx)); 
        
        step_cost = step_cost + energy_repair; % Add high power cost
        repair_count = repair_count + 1;
    end
    
    % Update Logs
    current_energy = current_energy + step_cost;
    log_energy(t) = current_energy;
    
    % C. Metrics (Ra and RMS)
    vals = abs(ground_map(:));
    log_Ra(t) = mean(vals);
    log_Rrms(t) = rms(vals);
end
fprintf('Done.\n');

% Calculate Final Statistics
duty_cycle = (repair_count / length(path)) * 100;

fprintf('--- MISSION SUMMARY ---\n');
fprintf('Total Energy: %.2f Joules\n', current_energy);
fprintf('Actuator Duty Cycle: %.2f%%\n', duty_cycle);
fprintf('Final RMS Roughness: %.4f mm\n', log_Rrms(end));

%% 4. THESIS PLOTTING
% FIGURE 1: Dynamic Topography
figure('Name', 'Randomized Field Results', 'Color', 'w', 'Position', [50, 50, 1000, 600]);

subplot(2,1,1);
surf(ground_truth_initial, 'EdgeColor', 'none'); 
title(['A. Initial Surface (Generated ' num2str(num_defects) ' Random Defects)']); 
zlabel('Height (mm)'); axis tight; axis equal; view(0, 90); colormap parula;

subplot(2,1,2);
surf(ground_map, 'EdgeColor', 'none'); 
title('B. Post-Curation (Restored)'); 
zlabel('Height (mm)'); axis tight; axis equal; view(0, 90); colormap parula;

% FIGURE 2: Technical Metrics
figure('Name', 'System Performance', 'Color', 'w', 'Position', [150, 150, 900, 600]);

% RMS Roughness
subplot(2,2,1);
plot(log_time, log_Rrms, 'r', 'LineWidth', 1.5); grid on;
title('Surface Roughness (RMS) Improvement'); 
ylabel('RMS (mm)'); xlabel('Time Steps');

% Energy Profile
subplot(2,2,2);
plot(log_time, log_energy, 'k', 'LineWidth', 1.5); grid on;
title('Cumulative Energy Consumption'); 
ylabel('Joules'); xlabel('Time Steps');
text(100, log_energy(end)*0.8, sprintf('Total: %.0f J', current_energy), ...
    'BackgroundColor', 'y', 'EdgeColor', 'k');

% Coverage Efficiency
subplot(2,2,[3 4]);
area(log_time, log_coverage, 'FaceColor', [0.2 0.7 0.3], 'FaceAlpha', 0.6); grid on;
title(sprintf('Coverage Efficiency (Duty Cycle: %.1f%%)', duty_cycle));
ylabel('Coverage %'); xlabel('Time Steps');
ylim([0 105]);