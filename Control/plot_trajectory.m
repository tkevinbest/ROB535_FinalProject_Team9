close all 
clear all
clc


load('TestTrack.mat')

heading = TestTrack.theta;
left_track = TestTrack.bl;
right_track = TestTrack.br;
center_line = TestTrack.cline;


target_path = [center_line;heading];

%Crop the first few points to match the graders initial conditions better
target_path = target_path(:,2:end);

num_points_prior = size(target_path,2);
interp_scale = 20;
num_points_post = interp_scale*num_points_prior;

original_size = 1:num_points_prior;
interp_size = 1:1/interp_scale:num_points_prior;


target_path_int = interp1(original_size,target_path',interp_size,'spline')';


sec_per_point = 0.7/interp_scale;

total_time = num_points_post*sec_per_point;

control_timestep = 0.01; 

t_span = 0:control_timestep:total_time;



[desired_velocity, desired_steer, expected_path] = path_at_t(t_span, sec_per_point, target_path_int,interp_scale);

% plot(t_span,desired_velocity);
% title("Desired velocity vs time");
% 
% figure
% 
% plot(t_span, desired_steer);
% title("Desried steer vs time");


%Calcualte the path 
%X,u,Y,v,psi,r
initial_state = [target_path(1,1);...
                0;...
                target_path(2,1);...
                0;...
                target_path(3,1);...
                0];

initial_state = [287;
                 5;
                 -176;
                 0;
                 2;
                 0];
             
             
             
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%                          %%%
%%%%  %%%%%%%%%%%%%%%%%%%%%%% %%%%
%%%%  %%%%%%%%%%%%%%%%%%%%%%% %%%%
 %%%                          %%%
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  
%Controller Gains
forward_proportional_gain = 1500;
%forward_bias = 72;
forward_bias = 130;

steering_proportional_gain = 5.5;
 
steer_lag = 33;


  %%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%                          %%%
%%%%  %%%%%%%%%%%%%%%%%%%%%%% %%%%
%%%%  %%%%%%%%%%%%%%%%%%%%%%% %%%%
 %%%                          %%%
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%


%define previous control
prev_control = [0,0];

%Set number of points to iterates
num_timesteps = size(t_span,2)-1;

%Set the intial states
states = zeros(num_timesteps,6);
states(1,:) = initial_state;


figure
hold on

scatter(left_track(1,:), left_track(2,:));
scatter(right_track(1,:), right_track(2,:));
for i = 1:num_timesteps
    current_state = states(i,:);
   
    
    %Since we are always lagging behind the set points, introduce a bit of
    %lag into steering to make sure we are not turning sooner than expected
    if i > steer_lag
        %Compare the desired heading and the current heading
        steering_angle_bias = desired_steer(i-steer_lag) - current_state(5);
        %Compare u and the desired forward velocity
        forward_velocity_error = desired_velocity(i-floor(steer_lag/20)) - current_state(2);
    else
        steering_angle_bias = desired_steer(i) - current_state(5);
        forward_velocity_error = desired_velocity(i) - current_state(2);

    end
    %Calculate control
    forward_control = forward_proportional_gain*forward_velocity_error+forward_bias;
    steering_control = steering_proportional_gain*steering_angle_bias;
    
    
    %Set the control input
    control_input = [prev_control;...
                     steering_control,forward_control];
                 
    prev_control = [steering_control,forward_control];
    
    
    %Forward integrate to get the new state
    [Y,~] = forwardIntegrateControlInput(control_input, current_state);
    
    %Assign state to control input
    states(i+1,:) = Y(end,:);
    
    %Plot the track and the positions
    
    
    
    if mod(i,500*sec_per_point/control_timestep) == 0
        scatter(states(1:i,1), states(1:i,3), 'r')
        scatter(expected_path(1:i,1), expected_path(1:i,2),'g')
        disp(i)
    end

end

scatter(states(:,1), states(:,3), 'r')
scatter(expected_path(:,1), expected_path(:,2),'g')


function[desired_velocity, desired_steer, expected_path] = path_at_t(t, sec_per_points, target_path,interp_scale)

last_point = size(target_path,2);

%Get the index of the past set point
past_point_idx = floor(t./sec_per_points)+1;

%Append a few points to the start to let the car catch up
past_point_idx = [ones(1,0*interp_scale+1),past_point_idx];


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