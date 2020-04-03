function run_simulation()
   close all; clc;

   %% Startup code
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

   h = youbot_init(vrep, id);
   h = youbot_hokuyo_init(vrep, h);
   pause(.2);

   %% Constants
   VISU_ON = true;

   %% Figures
   if VISU_ON
      figure_map = figure;
   end

   %% Finite state machine

   % Map constants
   L = 0.5;                      % Length of the squares
   OBSTACLE_THRESHOLD = 0;       % Threshold for votes

   % Finite state machine variables
   fsm = 'front_capture';

   % Robot relative location
   robot_x = 0;
   robot_y = 0;
   robot_a = 0;   % Counts the number of +90° rotations, must be in [0, 3]

   % Trajectories and robot absolute location
   position_y  = [];
   speed_y     = [];
   translation = false;
   position_a  = [];
   speed_a     = [];
   rotation    = false;
   robot_ax    = 0;
   robot_ay    = 0;
   robot_aa    = 0;
   robot_0x    = 0;
   robot_0y    = 0;
   robot_0a    = 0;
   x0 = 0;
   y0 = 0;

   % Absolute time
   abs_timer = tic;

   % Speeds, current rotation angle and trajectory constants
   vx = 0;
   vy = 0;
   va = 0;
   a = pi / 2;
   V_ROT = 0.5;
   V_TRANS = 0.3;
   K = 0.1;
   ALPHA = 0.9;   % 0.9
   Ts = 0.005;
   EPS_TRANS = 5 / 1000;     % 5mm
   EPS_ROT   = 1 / 180 * pi; % 1°
   t0 = 0;
   time = 0;
   dt = 0;
   rot_step = 0;

   % Travel trajectory variables
   path = [];
   current_step = 1;
   travel = false;

   % Map
   map = DMatrix;
   mem = [];

   % Gets robot initial absolute coordinates
   [res, pos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
   vrchk(vrep, res, true);
   robot_0x = pos(1);
   robot_0y = pos(2);
   [res, ang] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
   vrchk(vrep, res, true);
   robot_0a = ang(3);
   x0 = cos(robot_0a)  * robot_0x + sin(robot_0a) * robot_0y;
   y0 = -sin(robot_0a) * robot_0x + cos(robot_0a) * robot_0y;

   while true
      loop_timer = tic;
      if strcmp(fsm, 'front_capture')
         % Reads depth sensor data
         [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);

         % Discretizes the contact points
         pts = pts(:, contacts);
         dis = round(pts(1:2, :) / L);

         % Counts number of hits for each cell
         votes = zeros(9, 1);
         votes(1)  = sum(and(dis(1, :) == 1 , dis(2, :) == 1));
         votes(2)  = sum(and(dis(1, :) == 0 , dis(2, :) == 1));
         votes(3)  = sum(and(dis(1, :) == -1, dis(2, :) == 1));
         votes(4)  = sum(and(dis(1, :) == 1 , dis(2, :) == 0));
         votes(5)  = sum(and(dis(1, :) == 0 , dis(2, :) == 0));
         votes(6)  = sum(and(dis(1, :) == -1, dis(2, :) == 0));
         votes(7)  = sum(and(dis(1, :) == 1 , dis(2, :) == -1));
         votes(8)  = sum(and(dis(1, :) == 0 , dis(2, :) == -1));
         votes(9)  = sum(and(dis(1, :) == -1, dis(2, :) == -1));

         % Thresholds the votes - 0 corresponds to a seen cell, 3 to a visited
         % cell, 4 to an obstacle (1 to weak checkpoint and 2 to strong checkpoint)
         votes = 4 * (votes > OBSTACLE_THRESHOLD);
         % The current location becomes visited if it isnt an obstacle
         votes(5) = 3 * (votes(5) ~= 4);

         % Gets the locations of the cells
         locs = getLocations(robot_x, robot_y, robot_a);

         for i = 1 : 9
            % Gets the values of the cells in the map
            [~, map_val] = map.get(locs(:, i));
            % Compares them to what has been observed
            if i == 5 || i > 6
               votes(i) = max(votes(i), map_val);
            elseif votes(i) ~= 4
               votes(i) = map_val;
            end

            % And update the value of the cell
            map = map.suggest(locs(:, i), votes(i));
         end

         % Sets next state
         fsm = 'trajectory_planner';
      elseif strcmp(fsm, 'trajectory_planner')
         % Gets robot absolute coordinates
         [res, pos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
         vrchk(vrep, res, true);
         robot_ax = pos(1);
         robot_ay = pos(2);
         [res, ang] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
         vrchk(vrep, res, true);
         robot_aa = ang(3);

         % Simply rotates to the left if the surrounding cells are not
         % all known
         % Checks if all the necessary cells are known
         locs = getLocations(robot_x, robot_y, robot_a);
         if ~map.containsAll(locs)
            % Rotate
            %[position_a, speed_a, ~] = make_trajectory_s(robot_aa, robot_aa + a, V_ROT, K, ALPHA, Ts);
            [position_a, speed_a] = make_trajectory(robot_aa, robot_aa + a, V_ROT, 0.9, Ts);
            robot_a = mod(robot_a + sign(a), 4);
            rotation = true;
            fsm = 'move';
         else
            [~, cells] = map.getAll(locs);
            if cells(8) ~= 4 && cells(8) ~= 3 && (rot_step == 0 || rot_step == 1)
               if rot_step == 1
                  rot_step = rot_step + 1;
               end

               % Go forward
               y = -sin(robot_aa) * robot_ax + cos(robot_aa) * robot_ay;
               %[position_y, speed_y, ~] = make_trajectory_s(y, y - L, V_TRANS, K, ALPHA, Ts);
               [position_y, speed_y] = make_trajectory(y, y - L, V_TRANS, 0.9, Ts);
               translation = true;
               fsm = 'analyze';
            elseif rot_step ~= 1
               if rot_step == 0
                  a = -a;
               end
               rot_step = mod(rot_step + 1, 3);
               %[position_a, speed_a, ~] = make_trajectory_s(robot_aa, robot_aa + a, V_ROT, K, ALPHA, Ts);
               [position_a, speed_a] = make_trajectory(robot_aa, robot_aa + a, V_ROT, 0.9, Ts);
               robot_a = mod(robot_a + sign(a), 4);
               rotation = true;
               fsm = 'analyze';
            else
               rot_step = 0;
               fsm = 'find_path';
            end
         end
      elseif strcmp(fsm, 'analyze')
         locations = getLocations(robot_x, robot_y, robot_a);
         [~, map_val] = map.getAll(locations);

         % Checks at the right
         if map_val(6) ~= 3 && map_val(6) ~= 4 && ((map_val(9) == 4 && map_val(8) ~= 4 && map_val(3) ~= 4) || ((map_val(3) == 4 && map_val(9) ~= 4 && map_val(2) ~= 4)))
            map = map.suggest(locations(:, 6), 2);
         elseif map_val(6) ~= 3 &&  map_val(6) ~= 4
            map = map.suggest(locations(:,6), 1);
         end
         % Checks at the left
         if map_val(4) ~= 3 && map_val(4) ~= 4 && ((map_val(7) == 4 && map_val(8) ~= 4 && map_val(1) ~= 4) || ((map_val(1) == 4 && map_val(7) ~= 4 && map_val(2) ~= 4)))
            map = map.suggest(locations(:, 4), 2);
         elseif map_val(4) ~= 3 && map_val(4) ~= 4
            map = map.suggest(locations(:, 4), 1);
         end

         fsm = 'move';
      elseif strcmp(fsm, 'move')
         % Gets robot absolute coordinates
         [res, pos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
         vrchk(vrep, res, true);
         robot_ax = pos(1);
         robot_ay = pos(2);
         [res, ang] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
         vrchk(vrep, res, true);
         robot_aa = ang(3);

         if t0 == 0
            t0 = toc(abs_timer);
         end
         time = toc(abs_timer) - t0;

         if rotation
            if sign(adist(position_a(1), position_a(end))) * adist(robot_aa, position_a(end)) <= EPS_ROT
               %fprintf('%f -> %f, %f <= %f (dist=%f)\n', position_a(1), position_a(end), sign(adist(position_a(1), position_a(end))) * adist(robot_aa, position_a(end)), EPS_ROT, adist(robot_aa, position_a(end)));
               va = 0;
               t0 = 0;
               time = 0;
               rotation = false;
               fsm = 'correction';
            else
                index = min(1 + round(time/Ts), length(speed_a));
                dist = adist(robot_aa, position_a(index));
                va = dist;
            end
         elseif translation
            y = -sin(robot_aa) * robot_ax + cos(robot_aa) * robot_ay;
            if sign(position_y(end) - position_y(1)) * (position_y(end) - y) <= EPS_TRANS
               vy = 0;
               t0 = 0;
               time = 0;
               translation = false;
               if robot_a == 0
                  robot_y = robot_y - 1;
               elseif robot_a == 1
                  robot_x = robot_x + 1;
               elseif robot_a == 2
                  robot_y = robot_y + 1;
               elseif robot_a == 3
                  robot_x = robot_x - 1;
               end
               fsm = 'correction';
            else
                index = min(1 + round(time/Ts), length(speed_y));
                dist = position_y(index) - y;
                vy = dist;
            end
         end
      elseif strcmp(fsm, 'correction')
         % Gets robot absolute coordinates
         [res, pos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
         vrchk(vrep, res, true);
         robot_ax = pos(1);
         robot_ay = pos(2);
         [res, ang] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
         vrchk(vrep, res, true);
         robot_aa = ang(3);

         % Robot locations taking rotation into account
         x = cos(robot_aa)  * robot_ax + sin(robot_aa) * robot_ay;
         y = -sin(robot_aa) * robot_ax + cos(robot_aa) * robot_ay;

         % Objective locations
         tmp_x = x0 + (robot_x) * L;
         tmp_y = y0 + (robot_y) * L;
         tmp_a = pi/2 * robot_a;
         obj_a = robot_0a + tmp_a;
         obj_x = cos(tmp_a)  * tmp_x + sin(tmp_a) * tmp_y;
         obj_y = -sin(tmp_a) * tmp_x + cos(tmp_a) * tmp_y;

         va = 0;  vx = 0;  vy = 0;

         % Applies correction if needed
         if abs(adist(robot_aa, obj_a)) > EPS_ROT
            va = adist(robot_aa, obj_a);
            %fprintf('[a] %f -> %f with %f\n', a, obj_a, va);
         elseif abs(obj_x - x) > EPS_TRANS
            vx = obj_x - x;
            %fprintf('[x] %f -> %f with %f\n', x, obj_x, vx);
         elseif abs(obj_y - y) > EPS_TRANS
            vy = obj_y - y;
            %fprintf('[y] %f -> %f with %f\n', y, obj_y, vy);
         elseif ~travel
            fsm = 'front_capture';
         else
            fsm = 'follow_path';
         end
      elseif strcmp(fsm, 'find_path')
         weak_x = map.coords_x(map.values == 1);
         weak_y = map.coords_y(map.values == 1);
         strong_x = map.coords_x(map.values == 2);
         strong_y = map.coords_y(map.values == 2);

         targets = [];
         if ~isempty(strong_x)
            targets = [strong_x; strong_y];
         elseif ~isempty(weak_x)
            targets = [weak_x; weak_y];
         end

         if ~isempty(targets)
            start = [robot_x + map.offset_x; robot_y + map.offset_y];
            grid = map.matrix(:, :, 1);
            grid = or(grid == 4, isnan(grid));
            path = find_path(start, targets, grid);
            fsm = 'follow_path';
         else
            fsm = 'finished';
         end
      elseif strcmp(fsm, 'follow_path')
         [res, pos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
         vrchk(vrep, res, true);
         robot_ax = pos(1);
         robot_ay = pos(2);
         [res, ang] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
         vrchk(vrep, res, true);
         robot_aa = ang(3);

         if current_step == size(path, 2)
            travel = false;
            current_step = 1;
            path = [];
            fsm = 'front_capture';
         else
            travel = true;
            from = path(:, current_step);
            to = path(:, current_step + 1);
            ds = to - from;

            fprintf('(%d, %d) -> (%d, %d)\n', robot_x, robot_y, to(1), to(2));

            if ds(1) == 1
               tmp_a = 1;
            elseif ds(1) == -1
               tmp_a = 3;
            elseif ds(2) == -1
               tmp_a = 0;
            elseif ds(2) == 1
               tmp_a = 2;
            else % Should never occur
               tmp_a = 0;
            end

            if tmp_a ~= robot_a
               da = tmp_a - robot_a;
               if abs(da) == 3
                  da = -sign(da);
               end
               robot_a = tmp_a;
               [position_a, speed_a] = make_trajectory(robot_aa, robot_aa + da * pi/2, V_ROT, 0.9, Ts);
               rotation = true;
               fsm = 'move';
            else
               current_step = current_step + 1;
               % Go forward
               y = -sin(robot_aa) * robot_ax + cos(robot_aa) * robot_ay;
               %[position_y, speed_y, ~] = make_trajectory_s(y, y - L, V_TRANS, K, ALPHA, Ts);
               [position_y, speed_y] = make_trajectory(y, y - L, V_TRANS, 0.9, Ts);
               translation = true;
               fsm = 'move';
            end
         end
      elseif strcmp(fsm, 'finished')
         break;
      else
         error('Unknown state %s.', fsm);
      end

      if strcmp(fsm, 'trajectory_planner') || strcmp(fsm, 'follow_path')
         % Shows the local environment
         if VISU_ON
            figure(figure_map);
            hf = gco;
            delete(hf);
            hold on;
            disp(map.matrix(:, :, 1));
            im = or(map.matrix(:, :, 1) ~= 4, isnan(map.matrix(:, :, 1)));
            locs_weak = [map.coords_x(map.values == 1); map.coords_y(map.values == 1)];
            locs_strong = [map.coords_x(map.values == 2); map.coords_y(map.values == 2)];
            locs_visited = [map.coords_x(map.values == 3); map.coords_y(map.values == 3)];
            imagesc(im);
            colormap(gray);
            l1 = plot(locs(2, 5) + map.offset_y, locs(1, 5) + map.offset_x, 'go', 'DisplayName', 'from');
            l2 = plot(locs(2, 8) + map.offset_y, locs(1, 8) + map.offset_x, 'ro', 'DisplayName', 'to');
            l3 = plot(locs_visited(2, :), locs_visited(1, :), 'y*', 'DisplayName', 'visited');
            l4 = plot(locs_weak(2, :), locs_weak(1, :), 'b*', 'DisplayName', 'weak checkpoint');
            l5 = plot(locs_strong(2, :), locs_strong(1, :), 'b+', 'DisplayName', 'strong checkpoint');
            if ~isempty(path)
               l6 = plot(path(2, :), path(1, :), 'rs', 'DisplayName', 'path');
               legend([l1 l2 l3 l4 l5 l6]);
            else
               legend([l1 l2 l3 l4 l5]);
            end
            hold off;
         end
      end

      h = youbot_drive(vrep, h, vy, vx, va);

      elapsed = toc(loop_timer);
      timeleft = .05 - elapsed;
      if timeleft > 0
         pause(timeleft);
      end
   end
end
