function [sol_2, FLAG_terminate] = ROB535_ControlProject_part2_Team9(TestTrack, Xobs_seen, curr_state)
% ROB535_ControlProject_part2_Team9
%%% Inputs
%   TestTrack
%   Xobs_seen
%   curr_state
%%% Outputs
%   sol_2 - Vector of control inputs which will be passed to forwardIntegrateControlInput
%           Must have enough control inputs with time step 0.01 for 0.5
%           seconds
%   FLAG_terminate - binary flag to stop simulation

% Vehicle parameters
m = 1400; g = 9.806; delta_max = [-0.5,0.5]; Fx_max = [-5000,5000];
Nw = 2; f = 0.01; Iz = 2667; a = 1.35; b = 1.45; By = 0.27;
Cy = 1.2; Dy = 0.7; Ey = -1.6; Shy = 0; Svy = 0; F_max = 0.7*m*g;

% Extract values from TestTrack data
heading = TestTrack.theta;
left_track = TestTrack.bl;
right_track = TestTrack.br;
center_line = TestTrack.cline;

target_path = [center_line; heading];

% Figure out the starting index, end index will always be the same
% idx_start = 2;
idx_start = knnsearch(center_line', [curr_state(1) curr_state(2)]); 
% idx_end = length(center_line);
idx_end = idx_start + 49;

% Get new path given current obstacles
target_path = avoidObstacles(center_line, Xobs_seen, idx_start, idx_end, TestTrack);

% Interpolation
num_points_prior = size(target_path,2);
interp_scale = 20;
num_points_post = interp_scale*num_points_prior;

original_size = 1:num_points_prior;
interp_size = 1:1/interp_scale:num_points_prior;

target_path_int = interp1(original_size,target_path',interp_size,'spline')';

sec_per_point = 1.0/interp_scale;

total_time = num_points_post*sec_per_point;

control_timestep = 0.01; 
dt = control_timestep;

t_span = 0:control_timestep:total_time;

[desired_velocity, desired_steer, expected_path] = path_at_t(t_span, sec_per_point, target_path_int,interp_scale);

% Define initial state
initial_state = curr_state;

% Controller gains
forward_proportional_gain = 3000;
%forward_bias = 72;
forward_bias = 0;

steering_proportional_gain = 7.5;
 
%steer_lag = -45;
steer_lag = 0;

%Integral accumulator
heading_accumulator = 0;
steering_integral_gain = 0.2;
heading_accumulator_saturation = 1;

%define previous control
prev_control = [0,0];

%Set number of points to iterates
num_timesteps = size(t_span,2)-1;

%Set the intial states
states = zeros(num_timesteps,6);
states(1,:) = initial_state;

figure
hold on

plot(left_track(1,:), left_track(2,:), 'r');
plot(right_track(1,:), right_track(2,:),'b');

ROB535_ControlProject_part1_input = zeros(num_timesteps,2);

for i = 1:num_timesteps
    current_state = states(i,:);
   
    %Since we are always lagging behind the set points, introduce a bit of
    %lag into steering to make sure we are not turning sooner than expected
    if i > steer_lag
        %Compare the desired heading and the current heading
        steering_angle_error = desired_steer(i-steer_lag) - current_state(5);
        %Compare u and the desired forward velocity
        forward_velocity_error = desired_velocity(i-floor(steer_lag/33)) - current_state(2);
        
        %Add integral gain to steering
        curr_heading = current_state(5);
        
        R = [cos(-curr_heading), -sin(-curr_heading);
             sin(-curr_heading), cos(-curr_heading)];
        
        cur_pos = [current_state(1);current_state(3)];
        exp_pos = [expected_path(i,1);expected_path(i,2)];
        rot_curr_pos = R * cur_pos;
        rot_exp_pos = R * exp_pos;
        
        %IF y of current_position is greater, you are to the right
        % if you are to the left, you want positive sign, therefore
        % subtract expected y from current y
        angle_sign = sign(rot_exp_pos(2) - rot_curr_pos(2));

        distance_from_center_line = ((cur_pos(1)-exp_pos(1))^2 ...
                                   + (cur_pos(2)- exp_pos(2))^2)^0.5;

        heading_accumulator = heading_accumulator + angle_sign*distance_from_center_line*dt;
        
        
        if heading_accumulator > heading_accumulator_saturation
            heading_accumulator = heading_accumulator_saturation;
        end
        if heading_accumulator < -heading_accumulator_saturation
            heading_accumulator = -heading_accumulator_saturation;
        end
        
    else
        steering_angle_error = desired_steer(i) - current_state(5);
        forward_velocity_error = desired_velocity(i) - current_state(2);

    end
 
    %Calculate control
    forward_control = forward_proportional_gain*forward_velocity_error+forward_bias;
    steering_control = steering_proportional_gain*steering_angle_error ...
                     + steering_integral_gain*heading_accumulator;
     
    %Saturation
    forward_control = max(min(forward_control,Fx_max(2)),Fx_max(1));
%     if forward_control > Fx_max(2)
%        forward_control = Fx_max(2);
%     end
% 
%     if forward_control < Fx_max(1)
%        forward_control = Fx_max(1);
%     end

    %Set constraints based total traction
    u = current_state(2);
    v = current_state(4);
    r = current_state(6);
    Fzr = a/(a+b)*m*g;
    alpha_r = rad2deg(-atan((v-b*r)/u));
    phi_yr = (1-Ey)*(alpha_r + Shy) + Ey/By*atan(By*(alpha_r+Shy));
    Fyr = Fzr*Dy*sin(Cy*atan(By*phi_yr)+Svy);
    F_total = sqrt(Nw*forward_control^2 + Fyr^2);
    
    
    if F_total >= F_max
%         
        F_x_max = sqrt(F_max^2 - Fyr^2)/Nw;
        
        forward_control = F_x_max;
        
        %forward_control = F_max/F_total*forward_control;
       
    end
    
    if forward_control > Fx_max(2)
        forward_control = Fx_max(2);
    end
   
    if forward_control < Fx_max(1)
        forward_control = Fx_max(1);
    end
   
    if steering_control > delta_max(2)
        steering_control = delta_max(2);
    end
   
    if steering_control < delta_max(1)
        steering_control = delta_max(1);
    end
    
    %Set the control input
    control_input = [prev_control;...
                     steering_control,forward_control];
                 
    prev_control = [steering_control,forward_control];
    
    ROB535_ControlProject_part1_input(i,:) = [steering_control,forward_control];
    
    %Forward integrate to get the new state
    [Y,~] = forwardIntegrateControlInput(control_input, current_state);
    
    %Assign state to control input
    states(i+1,:) = Y(end,:);
    
    %Plot the track and the positions

    if mod(i,50*sec_per_point/control_timestep) == 0
%         scatter(states(1:i,1), states(1:i,3), 'c')
%         scatter(expected_path(1:i,1), expected_path(1:i,2),'g')
%         for k = 1:Nobs
%             obj = line([Xobs{k}(:,1)', Xobs{k}(1,1)],[Xobs{k}(:,2)',Xobs{k}(1,2)]);
%             set(obj,'LineWidth',1,'Color','r');
%         end
        disp(i)
        
    end

end

plot(states(:,1), states(:,3), 'c')
plot(expected_path(:,1), expected_path(:,2),'g')

for k = 1:Nobs
    obj = line([Xobs{k}(:,1)', Xobs{k}(1,1)],[Xobs{k}(:,2)',Xobs{k}(1,2)]);
    set(obj,'LineWidth',1,'Color','r');
end


figure
plot(sqrt(states(:,2).^2 + states(:,4).^2))
hold on
plot(desired_velocity)
legend('actual velocity', 'desired velocity')
title("Velocity vs iteration")
figure
plot(states(:,5))
hold on 
plot(desired_steer)
legend('actual steer', 'desired steer')
title("Steering vs iteration")

path = [states(:,1),states(:,3)];

getTrajectoryInfo(path,ROB535_ControlProject_part1_input,Xobs)

save('ROB535_ControlProject_part1_input.mat','ROB535_ControlProject_part1_input')


% dt = 0.01;
% sol_2 = ones(0.5/dt,2);
% 
% FLAG_terminate = true;

end

%% Supporting functions
% Avoid obstacles
function route = avoidObstacles(curr_route, Xobs, traj_idx_start, traj_idx_end, TestTrack)

left_track = TestTrack.bl;
right_track = TestTrack.br;
route = TestTrack.cline;

traj_idx = traj_idx_start;
curr_point = curr_route(:,traj_idx);
prev_point = curr_route(:,traj_idx-1);

obj_dectected = false; % tracks if object is dectected
obj_location = zeros(size(Xobs{1})); % will store location of object that is detected

tolerance = 0.8; % tolerance to determine how close object needs to be on trajectory to trigger evasive maneuvers
tolerance_obj_dist = 15; % tolerance to determine how close object needs to be on trajectory to trigger evasive maneuvers
object_avoid_buff = 4;

num_points = 10; % number of points to create between two points in trajectory
traj_idx = 2; % stores where we are in trajectory
traj_idx_increment = 1; % stores how much to increase trajectory counter by

j = 1; %object number CHANGE WHEN PASSING ONE OBJ

while traj_idx < traj_idx_end
    curr_point = curr_route(:,traj_idx);
    
    % %%%%%%% Detect Collisions %%%%%%%
    % interpolate line between current point and previous
    x_vector = linspace(prev_point(1),curr_point(1),num_points);
    y_vector = linspace(prev_point(2),curr_point(2),num_points);
    
    for i = 1:num_points-1
        % determine line between consecutive points on interpolated line
        slope = (y_vector(i) - y_vector(i+1))/(x_vector(i) - x_vector(i+1));
        intercept = y_vector(i) - slope * x_vector(i);
        
        % dectect if current location collides with any object on trajectory
        if j <= size(Xobs,2)
            for k = 1:4 % check all 4 corners of object for impact with center line
                distance_to_obj = sqrt((y_vector(i) - Xobs{j}(k,2))^2 + (x_vector(i) - Xobs{j}(k,1))^2);%add in a distance to next object being less than the 10 steps
                obj_line = abs(abs(Xobs{j}(k,2)) - abs(slope*Xobs{j}(k,1) + intercept));
                if distance_to_obj < tolerance_obj_dist &&  obj_line <= tolerance
                    obj_dectected = true;
                    obj_location = Xobs{j};
                    continue
                end
                
            end
        end
    end
    
    % if object is detected plan evasive maneuvers
    if obj_dectected
        disp(['Object ' num2str(j) ' Detected'])
        % %%%%%%% Find min distance between closest points on right/left that are closest to object and object %%%%%%%
        % find closest points on left and right side of track respectively to detected object
                obs_center = center_obs(obj_location);

        [~, dist_right] = knnsearch(right_track(:,traj_idx)', obs_center);
        [~, dist_left] = knnsearch(left_track(:,traj_idx)', obs_center);

        % %%%%%%% Avoid object %%%%%%%
        % assign whether object is located on left or right side, determined by which minimum distance is smallest
        if dist_right < dist_left % if object is on right
            route(:,traj_idx:traj_idx+object_avoid_buff) = (left_track(:,traj_idx:traj_idx+object_avoid_buff)+curr_route(:,traj_idx:traj_idx+object_avoid_buff)) ./ 2;
            disp(['Zagged RIGHT to avoid Object ' num2str(j)])
            
        else % if object is on left
            route(:,traj_idx:traj_idx+object_avoid_buff) = (right_track(:,traj_idx:traj_idx+object_avoid_buff)+curr_route(:,traj_idx:traj_idx+object_avoid_buff)) ./ 2;
            disp(['Zigged LEFT to avoid Object ' num2str(j)])
        end
        
        j = j+1; %onto next object!
    end
    
    % %%%%%%% reset/update values %%%%%%%
%     plot(route(1,1:traj_idx),route(2,1:traj_idx),'b')
    obj_dectected = false;
    obj_location = zeros(size(Xobs{1}));
    
    prev_point = curr_point;
    traj_idx = traj_idx + traj_idx_increment;
    
    
end
% Set Heading
heading = atan2(diff(route(2,:)),diff(route(1,:)));
heading(end+1) = heading(end);

route(3,:) = heading;

end

function point = center_obs(obs)
    point(1) = mean(obs(:,1));
    point(2) = mean(obs(:,2));
end

function[desired_velocity, desired_steer, expected_path] = path_at_t(t, sec_per_points, target_path,interp_scale)

%Calibration variable - repeats first velocity so car can get up to speed
initial_buffer = 0;


%Understand where you want to end up 
last_point = size(target_path,2);


%Get the index of the past set point
past_point_idx = floor(t./sec_per_points)+1;


%Append a few points to the start to let the car catch up
past_point_idx = [ones(1,initial_buffer*interp_scale+1),past_point_idx];


%Make sure that you do not go out of bounds
past_point_idx(:,past_point_idx > last_point-1) = last_point-1;


%Add points int the end to make sure that you get to the last point
past_point_idx = [past_point_idx, (last_point-1)*ones(1,200)];


%Get the index of the future set point
set_point_idx = past_point_idx + 1;


position_start = target_path(1:2,past_point_idx);
position_end = target_path(1:2,set_point_idx); 

desired_velocity = vecnorm(position_end - position_start)./sec_per_points;

desired_steer = target_path(3,past_point_idx);

expected_path = target_path(:,past_point_idx)';

end