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


%%%%%%%%%%%%%%%%%%%%%%
% Vehicle parameters %
%%%%%%%%%%%%%%%%%%%%%%



%Vehicle parameters
m = 1400;
g = 9.806;
delta_max = [-0.5,0.5];
Fx_max = [-5000,5000];
Nw = 2;
f = 0.01;
Iz = 2667;
a = 1.35;
b = 1.45;
By = 0.27;
Cy = 1.2;
Dy = 0.7;
Ey = -1.6;
Shy = 0;
Svy = 0;
F_max = 0.7*m*g;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Truncate target path based on current position %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Crop based on idx_start
heading = TestTrack.theta;
left_track = TestTrack.bl;
right_track = TestTrack.br;
center_line = TestTrack.cline;

%Generate the target path
target_path = [center_line;heading];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate new center line based on seen obstacles   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Get obstacles
Xobs = Xobs_seen;
Nobs = size(Xobs,2);

%Get idx_end
idx_end = length(target_path);

%Get new path
target_path = avoidObstacles(center_line, Xobs,2,idx_end,left_track,right_track,heading);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Interpolate target path based on current position %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Interpolate the target path
num_points_prior = size(target_path,2);
interp_scale = 20;
num_points_post = interp_scale*num_points_prior;

%Get interpolation size vectors 
original_size_vector = 1:num_points_prior;
interp_size_vector = 1:1/interp_scale:num_points_prior;

%Get the interpolrated path
target_path_int = interp1(original_size_vector,target_path',interp_size_vector,'spline')';



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create reference velocity, steering and path over time %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%Define the time it takes to go from one point to another (in the original,
%276 point array)
sec_per_point = 1.4/interp_scale;

%Calculate the total time in seconds in takes to run the course
total_time = num_points_post*sec_per_point;

%Set the controller loop rate
control_timestep = 0.01; 
dt_ = control_timestep;

%Set the total time
t_span = 0:control_timestep:total_time*1.02;


%Calculate the reference trajectories for velocity and steering with
%expected positions
[desired_velocity, desired_steer, expected_path] = path_at_t(t_span, sec_per_point, target_path_int,interp_scale);


%Set the initial state as the current state fed in to the controller
initial_state = curr_state;
             
       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control config, fancy boxes to find it in code easier  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


             
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%                          %%%
%%%%  %%%%%%%%%%%%%%%%%%%%%%% %%%%
%%%%  %%%%%%%%%%%%%%%%%%%%%%% %%%%
 %%%                          %%%
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  
%Controller Gains
forward_proportional_gain = 3000;
%forward_bias = 72;
forward_bias = 0;

steering_proportional_gain = 7.5;
 
%steer_lag = -45;
steer_lag = 0;

%Path generation


%Integral accumulator
heading_accumulator = 0;
steering_integral_gain = 0.8;
heading_accumulator_saturation = 2;


  %%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%                          %%%
%%%%  %%%%%%%%%%%%%%%%%%%%%%% %%%%
%%%%  %%%%%%%%%%%%%%%%%%%%%%% %%%%
 %%%                          %%%
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%


%define previous control
prev_control = [0,0];

%Set number of points to iterates
control_time = 0.5;
num_timesteps = control_time/dt_;

%Set the intial states
states = zeros(num_timesteps,6);
states(1,:) = initial_state;

control = zeros(num_timesteps+1,2);

%%%%%%%%%%%%%%%%%%%%%
% Main control loop %
%%%%%%%%%%%%%%%%%%%%%


%Calculate the new path based on the current position
desired_center_line = expected_path(:,1:2)';
x = curr_state(1);
y = curr_state(3);

idx_start = knnsearch(desired_center_line',[x,y]) + 1;

%Start a little bit ahead of the initial point
if(idx_start == 1)
    idx_start = 2;
end 

percent_done = idx_start/size(expected_path,1)

idx_expected_path = idx_start:idx_start + num_timesteps;

for i =  1:num_timesteps
    
    kk = idx_expected_path(i);
    
    current_state = states(i,:);
    
    curr_desired_steer = desired_steer(kk);
    curr_desired_velocity = desired_velocity(kk);
    
    %Compare the desired heading and the current heading
    steering_angle_error = curr_desired_steer - current_state(5);
    %Compare u and the desired forward velocity
    forward_velocity_error = curr_desired_velocity - current_state(2);


    %Add integral gain to steering
    curr_heading = current_state(5);

    R = [cos(-curr_heading), -sin(-curr_heading);
         sin(-curr_heading), cos(-curr_heading)];

    cur_pos = [current_state(1);current_state(3)];
    exp_pos = [expected_path(kk,1);expected_path(kk,2)];
    rot_curr_pos = R * cur_pos;
    rot_exp_pos = R * exp_pos;

    %IF y of current_position is greater, you are to the right
    % if you are to the left, you want positive sign, therefore
    % subtract expected y from current y
    angle_sign = sign(rot_exp_pos(2) - rot_curr_pos(2));

    distance_from_center_line = ((cur_pos(1)-exp_pos(1))^2 ...
                               + (cur_pos(2)- exp_pos(2))^2)^0.5;

    heading_accumulator = heading_accumulator + angle_sign*distance_from_center_line*dt_;

    
    

    if heading_accumulator > heading_accumulator_saturation
        heading_accumulator = heading_accumulator_saturation;
    end
    if heading_accumulator < -heading_accumulator_saturation
        heading_accumulator = -heading_accumulator_saturation;
    end
   
    
    %Calculate control
    forward_control = forward_proportional_gain*forward_velocity_error+forward_bias;
    steering_control = steering_proportional_gain*steering_angle_error ...
                     + steering_integral_gain*heading_accumulator;
    
    
    
    %Saturation
    if forward_control > Fx_max(2)
       forward_control = Fx_max(2);
    end

    if forward_control < Fx_max(1)
       forward_control = Fx_max(1);
    end

    
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
   
    %Total saturation
    
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
    
    control(i+1,:) = [steering_control,forward_control];
        
    %Forward integrate to get the new state
    [Y,~] = forwardIntegrateControlInput(control_input, current_state);
    
    %Assign state to control input
    states(i+1,:) = Y(end,:);
    
    
end

%Plot the track and the positions

% 
% figure(1)
% clf(1)
% plot(idx_expected_path, sqrt(states(:,2).^2 + states(:,4).^2))
% hold on
% plot(desired_velocity)
% legend('actual velocity', 'desired velocity')
% title("Velocity vs iteration")
% 
% 
% figure(2)
% clf(2)
% plot(idx_expected_path,  states(:,5))
% hold on 
% plot(desired_steer)
% legend('actual steer', 'desired steer')
% title("Steering vs iteration")
% 
% Output
% figure(3)
% hold on 
% 
% plot(left_track(1,:), left_track(2,:), 'r');
% plot(right_track(1,:), right_track(2,:),'b');
% plot(states(:,1), states(:,3), 'c')
% expcted_path_plot = plot(expected_path(:,1), expected_path(:,2),'g--')
% for v = 1:Nobs
% obj = line([Xobs{v}(:,1)', Xobs{v}(1,1)],[Xobs{v}(:,2)',Xobs{v}(1,2)]);
% set(obj,'LineWidth',1,'Color','r');
% end
% disp(i)
% delete(expcted_path_plot)


sol_2 = control;
if percent_done > 0.97
    FLAG_terminate = true;
else
    FLAG_terminate = false;

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



function route = avoidObstacles(curr_route, Xobs, traj_idx_start, traj_idx_end, left_track_ao, right_track_ao,original_heading)
    route = curr_route;

    traj_idx = traj_idx_start;
    curr_point = curr_route(:,traj_idx);
    prev_point = curr_route(:,traj_idx-1);
    
    %There is no ostacle
        
    if size(Xobs) == 0
        route = [curr_route;original_heading];
    else
        obj_dectected = false; % tracks if object is dectected
        obj_location = zeros(size(Xobs{1})); % will store location of object that is detected

        tolerance = 0.8; % tolerance to determine how close object needs to be on trajectory to trigger evasive maneuvers
        %CHANGE
        tolerance_obj_dist = 10; % tolerance to determine how close object needs to be on trajectory to trigger evasive maneuvers
        object_avoid_buff = 1;

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

            for ii = 1:num_points-1
                % determine line between consecutive points on interpolated line
                slope = (y_vector(ii) - y_vector(ii+1))/(x_vector(ii) - x_vector(ii+1));
                intercept = y_vector(ii) - slope * x_vector(ii);

                % dectect if current location collides with any object on trajectory
                if j <= size(Xobs,2)
                    for k = 1:4 % check all 4 corners of object for impact with center line
                        distance_to_obj = sqrt((y_vector(ii) - Xobs{j}(k,2))^2 + (x_vector(ii) - Xobs{j}(k,1))^2);%add in a distance to next object being less than the 10 steps
                        obj_line = abs(abs(Xobs{j}(k,2)) - abs(slope*Xobs{j}(k,1) + intercept));
                        if distance_to_obj < tolerance_obj_dist %&&  obj_line <= tolerance %%CHANGE
                            obj_dectected = true;
                            obj_location = Xobs{j};
                            continue
                        end

                    end
                end
            end

            % if object is detected plan evasive maneuvers
            if obj_dectected
%                 disp(['Object ' num2str(j) ' Detected'])
                % %%%%%%% Find min distance between closest points on right/left that are closest to object and object %%%%%%%
                % find closest points on left and right side of track respectively to detected object
                        obs_center = center_obs(obj_location);

                [idx_right, dist_right] = knnsearch(right_track_ao', obs_center);
                [idx_left, dist_left] = knnsearch(left_track_ao', obs_center);

                % %%%%%%% Avoid object %%%%%%%
                % assign whether object is located on left or right side, determined by which minimum distance is smallest
                if dist_right < dist_left % if object is on right
                    range = max(idx_right,1):idx_right+object_avoid_buff;
                    route(:,range) = (left_track_ao(:,range)+curr_route(:,range)) ./ 2;
%                     disp(['Zagged RIGHT to avoid Object ' num2str(j)])

                else % if objec(t is on left
                    range = max(idx_left,1):idx_left+object_avoid_buff;
                    route(:,range) = (right_track_ao(:,range)+curr_route(:,range)) ./ 2;
%                     disp(['Zigged LEFT to avoid Object ' num2str(j)])
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
        heading_ao = atan2(diff(route(2,:)),diff(route(1,:)));
        heading_ao(end+1) = heading_ao(end);

        route(3,:) = heading_ao;
        
    end
end
        
    

function point = center_obs(obs)
    point(1) = mean(obs(:,1));
    point(2) = mean(obs(:,2));
end
    
        


end