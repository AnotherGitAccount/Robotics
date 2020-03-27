function run_simulation()
    close all; clc;
    
 %% Figures 
    nav = figure;
 
 %% Initiate the connection to the simulator. 
    
    disp('Program started');
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
  
    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);

    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

    % Retrieve all handles, and stream arm and wheel joints, the robot's pose, the Hokuyo, and the arm tip pose.
    % The tip corresponds to the point between the two tongs of the gripper (for more details, see later or in the 
    % file focused/youbot_arm.m). 
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);
    pause(.2);

   %% Constants
    TIMESTEP = .05;
    % Space discretization step
    DS = 0.01;
    % Depth sensor range
    SENSOR_RANGE = 5;
    % Max rotation speed
    V_ROT = 4 * 0.07;
    % K-factor
    K = 0.3;
    % ALPHA-factor
    ALPHA = 0.9;
    % Number of chunks per image (x axis)
    U = 4;
    % Number of chunks per image (y axis)
    V = 4;
    
   %% "Global" variables
    % Finite state machine state
    fsm = ["capture_area" 'read_depth'];
    % Step for state find_obstacle
    step = 0;
    % Relative map - contains all in-range depth measures around the robot
    rel_map = [];
    % u and v state variables for find_obstacles
    u = 0;
    v = 0;
    
    %% Start demo
    while true
        tic
        
        if vrep.simxGetConnectionId(id) == -1
            error('Lost connection to remote API.');
        end
     
        % Position and orientation of the robot
        [res, pos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, ang] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        
        sx = 0; sy = 0; st = 0;
        
      %% State machine
         if strcmp(fsm(1), 'capture_area')
            if strcmp(fsm(2), 'read_depth')
                % Inits the relative map if it is the first step
                if step == 0
                    rel_map = false(2 * SENSOR_RANGE / DS, 2 * SENSOR_RANGE / DS);
                end
                
                if step ~= 4
                    % Read depth sensor and evaluates the closest obstacle
                    [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
                    % Removing out-of-range measures and keeps x, y only
                    pts = pts(1:2, contacts);

                    % Rotating points
                    for i = 1 : size(pts, 2)
                        pts(:, i) = [cos(ang(3)) -sin(ang(3)); sin(ang(3)) cos(ang(3))] * pts(:, i) + [5; 5];
                        % Adds the point in the relative map after discretizing
                        % its coordinates
                        rel_map(max(round(pts(1, i) / DS), 1), max(round(pts(2, i) / DS), 1)) = true;
                    end
                end
                
                step = step + 1;
                if step == 5
                    step = 0;
                    save 'fog.mat' rel_map;
                    fsm(1) = 'find_obstacles';
                    fsm(2) = 'dilation';
                else
                    % Computes the trajectory for the rotation
                    [s_theta, sp_theta, spp_theta] = make_trajectory_s(ang(3), ang(3) + pi / 2, V_ROT, K, ALPHA, 10000);
                    fsm(2) = 'rotate';
                end
            elseif strcmp(fsm(2), 'rotate')
                [~, index] = min(adist(ang(3), s_theta));
                if index == 10000
                    st = 0;
                    fsm(2) = 'read_depth';
                elseif index == 1
                    st = sp_theta(index + 1);
                else
                    st = sp_theta(index);
                end
            end
         elseif strcmp(fsm(1), 'find_obstacles')
             if strcmp(fsm(2), 'dilation')
                tmp = bwmorph(rel_map, 'dilate', 6);
                fsm(2) = 'erosion';
             elseif strcmp(fsm(2), 'erosion')
                tmp = bwmorph(tmp, 'erode', 5);
                fsm(2) = 'hough';
             elseif strcmp(fsm(2), 'hough')
                [H, T, R] = hough(tmp);
                fsm(2) = 'peaks';
             elseif strcmp(fsm(2), 'peaks')
                P  = houghpeaks(H, 5, 'threshold', ceil(0.3 * max(H(:))));
                fsm(2) = 'lines';
             elseif strcmp(fsm(2), 'lines')
                lines = houghlines(tmp, T, R, P, 'FillGap', 10, 'MinLength', 30);
                figure(nav);
                hold on
                fsm(2) = 'circles';
             elseif strcmp(fsm(2), 'circles')
                img_length = 2 * SENSOR_RANGE / DS;
                u0 = max(round(u * img_length / U), 1);
                uf = round((u + 1) * img_length / U);
                v0 = max(round(v * img_length / V), 1);
                vf = round((v + 1) * img_length / V);

                 [centers, radii] = imfindcircles(rel_map(u0:uf, v0:vf), [25 35], 'Sensitivity', 0.95);

                 if ~isempty(centers)
                    centers = centers + [v0 u0];
                 end

                 viscircles(centers, radii);

                 if u < U - 1
                     u = u + 1;
                 else
                     u = 0;
                     v = v + 1;
                 end

                 if v == V
                     for k = 1:length(lines)
                        xy = [lines(k).point1; lines(k).point2];
                        plot(xy(:, 1), xy(:, 2),'LineWidth', 2, 'Color', 'green');

                        % Plot beginnings and ends of lines
                        plot(xy(1, 1), xy(1, 2), 'x', 'LineWidth', 2, 'Color', 'yellow');
                        plot(xy(2, 1), xy(2, 2), 'x', 'LineWidth', 2, 'Color', 'red');
                     end
                     hold off;
                     fsm(1) = 'finished';
                     fsm(2) = 'none';
                 end
             else
                 error('Unknown state %s.', fsm);
             end
         elseif strcmp(fsm(1), 'finished') 
             break;
         else
            error('Unknown state %s.', fsm);
         end

         h = youbot_drive(vrep, h, sy, sx, st);
         
         elapsed = toc;
         timeleft = TIMESTEP - elapsed;
         if timeleft > 0      
             pause(min(timeleft, .01));
         else
             fprintf("Too long before : %s %s -> %f\n", fsm(1), fsm(2), elapsed);
         end
    end

    pause(3);
    
end

