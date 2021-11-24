close all 
clear

%% Load/plot track and obstacles
load('TestTrack.mat')

heading = TestTrack.theta;
left_track = TestTrack.bl;
right_track = TestTrack.br;
center_line = TestTrack.cline;

% generate obstacles
Nobs = 10; % number of obstacles
Xobs = generateRandomObstacles(Nobs);

% plot track and obstacles
figure(1)
hold on
plot(left_track(1,:), left_track(2,:),'k');
plot(right_track(1,:), right_track(2,:),'k');
plot(center_line(1,:), center_line(2,:),'--k');

for i = 1:Nobs
    obj = line([Xobs{i}(:,1)', Xobs{i}(1,1)],[Xobs{i}(:,2)',Xobs{i}(1,2)]);
    set(obj,'LineWidth',1,'Color','r');
end

%% Plan/plot route around obstacles

% define parameters
route = center_line; % vector to store route; initialize to be the centerline
prev_point = center_line(:,1); % tracks previous point on center line

obj_dectected = false; % tracks if object is dectected
obj_location = zeros(size(Xobs{1})); % will store location of object that is detected

tolerance = 0.75; % tolerance to determine how close object needs to be on trajectory to trigger evasive maneuvers 

num_points = 10; % number of points to create between two points in trajectory
traj_idx = 2; % stores where we are in trajectory
traj_idx_increment = 1; % stores how much to increase trajectory counter by

% plan route
while traj_idx < length(center_line)
    curr_point = center_line(:,traj_idx); 
    
    % %%%%%%% Detect Collisions %%%%%%%
    % interpolate line between current point and previous
    x_vector = linspace(prev_point(1),curr_point(1),num_points);
    y_vector = linspace(prev_point(2),curr_point(2),num_points);
    
    for i = 1:num_points-1
        % determine line between consecutive points on interpolated line
        slope = (y_vector(i) - y_vector(i+1))/(x_vector(i) - x_vector(i+1));
        intercept = y_vector(i) - slope * x_vector(i);
        
        % dectect if current location collides with any object on trajectory
        for j = 1:Nobs
            for k = 1:4 % check all 4 corners of object for impact with center line
                if abs(abs(Xobs{j}(k,2)) - abs(slope*Xobs{j}(k,1) + intercept)) <= tolerance
                    obj_dectected = true;
                    obj_location = Xobs{j};
                end
            end
        end
    end
    
    % if object is detected plan evasive maneuvers 
    if obj_dectected 
        % %%%%%%% Find min distance between closest points on right/left that are closest to object and object %%%%%%%
        % find closest points on left and right side of track respectively to detected object
        idx_right = knnsearch(right_track(:,traj_idx:traj_idx+3)', obj_location);
        idx_left = knnsearch(left_track(:,traj_idx-1:traj_idx+3)', obj_location);
        
        point_right1 = right_track(:,traj_idx + mode(idx_right) - 1);
        point_right2 = right_track(:,traj_idx + mode(idx_right));
        point_left1 = left_track(:,traj_idx + mode(idx_left) - 1);
        point_left2 = left_track(:,traj_idx + mode(idx_left));
        
        % determine line between consecutive points on left and right line
        slope_right = (point_right1(2) - point_right2(2))/(point_right1(1) - point_right2(1));
        intercept_right = point_right1(2) - slope_right * (point_right2(1));
        
        slope_left = (point_left1(2) - point_left2(2))/(point_left1(1) - point_left2(1));
        intercept_left = point_left1(2) - slope_left * (point_left1(1));
        
        % find closest intercept on object for right and left lines
        min_dist_right = inf;
        min_dist_left = inf;
        for k = 1:4 % check all 4 corners of object for impact with center line
            if abs(abs(obj_location(k,2)) - abs(slope_right*obj_location(k,1) + intercept_right)) < min_dist_right
                min_dist_right = abs(abs(obj_location(k,2)) - abs(slope_right*obj_location(k,1) + intercept_right));
            end
            
            if abs(abs(obj_location(k,2)) - abs(slope_left*obj_location(k,1) + intercept_left)) < min_dist_left
                min_dist_left = abs(abs(obj_location(k,2)) - abs(slope_left*obj_location(k,1) + intercept_left));
            end
        end
        
        % %%%%%%% Avoid object %%%%%%%
        % assign whether object is located on left or right side, determined by which minimum distance is smallest
        if min_dist_right < min_dist_left % if object is on right
            route(:,traj_idx) = (left_track(:,traj_idx)+center_line(:,traj_idx)) ./ 2;
        else % if object is on left
            route(:,traj_idx) = (right_track(:,traj_idx)+center_line(:,traj_idx)) ./ 2; 
        end

    end
    
    % %%%%%%% reset/update values %%%%%%%
    obj_dectected = false;
    obj_location = zeros(size(Xobs{1}));

    prev_point = curr_point;
    traj_idx = traj_idx + traj_idx_increment;
    
end

% plot final route
plot(route(1,:),route(2,:),'b')