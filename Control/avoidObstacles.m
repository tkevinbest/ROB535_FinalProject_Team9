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

