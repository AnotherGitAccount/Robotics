function run_simulation()
    close all;
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
    V_ROT = 4 * 0.08;
    % K-factor
    K = 0.3;
    % ALPHA-factor
    ALPHA = 0.9;
    
   %% "Global" variables
    % Finite state machine state
    fsm = ["capture_area" 'read_depth'];
    % Step for state find_obstacle
    step = 0;
    % Relative map - contains all in-range depth measures around the robot
    rel_map = [];
    
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
                        rel_map(round(pts(1, i) / DS), round(pts(2, i) / DS)) = true;
                    end
                end
                
                step = step + 1;
                if step == 5
                    step = 0;
                    fsm(1) = 'finished';
                    fsm(2) = 'none';
                else
                    % Computes the trajectory for the rotation
                    [s_theta, sp_theta, spp_theta] = make_trajectory_s(ang(3), ang(3) + pi / 2, V_ROT, K, ALPHA, 10000);
                    figure
                    plot(spp_theta);
                    fsm(2) = 'rotate';
                end
            elseif strcmp(fsm(2), 'rotate')
                % Computes the index of the current angle in the array,
                % uses some funcky distance measure but it works!
                % dist(a, b) = min { |a - b|, 2pi - |a - b| }
                [~, index] = min(min(abs(ang(3) - s_theta), 2 * pi - abs(ang(3) - s_theta)));
                if index == 10000
                    st = 0;
                    fsm(2) = 'read_depth';
                elseif index == 1
                    st = sp_theta(index + 1);
                else
                    st = sp_theta(index);
                end
            end
         elseif strcmp(fsm(1), 'finished') 
             figure
             imagesc(rel_map);
             break;
         else
            error('Unknown state %s.', fsm);
         end

         h = youbot_drive(vrep, h, sy, sx, st);
         
         elapsed = toc;
         timeleft = TIMESTEP - elapsed;
         if timeleft > 0
             pause(min(timeleft, .01));
         end
    end

    pause(3);
    
end

