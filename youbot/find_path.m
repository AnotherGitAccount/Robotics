function locations = find_path(start, targets, matrix)
   G = make_graph(matrix);

   [size_x, size_y] = size(matrix);
   start_node = (start(2) - 1) * size_x + start(1);

   best_path = [];
   path_size = inf;

   for i = 1 : size(targets, 2)
      target = targets(:, i);
      target_node = (target(2) - 1) * size_x + target(1);
      path = shortestpath(G, start_node, target_node);
      if length(path) < path_size
         best_path = path;
         path_size = length(path);
      end
   end

   xs = 1 + mod(best_path - 1, size_x);
   ys = 1 + ((best_path - xs) / size_x);
   locations = [xs; ys];
end
