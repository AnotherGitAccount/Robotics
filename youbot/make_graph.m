function G = make_graph(matrix)
   % matrix is a binary matrix, 0 if empty, 1 if obstacle
   [size_x, size_y] = size(matrix);
   x_s = 1 : size_x;
   y_s = 1 : size_y;
   disp(size_x);
   disp(size_y);
   adj_mat = zeros(size_x * size_y);

   for x = x_s
      for y = y_s
         u = (y - 1) * size_x + x;
         if x + 1 <= size_x
            v = (y - 1) * size_x + x + 1;
            adj_mat(v, u) = ~(matrix(x + 1, y) || matrix(x, y));
         end
         if x - 1 > 0
            v = (y - 1) * size_x + x - 1;
            adj_mat(v, u) = ~(matrix(x - 1, y) || matrix(x, y));
         end
         if y + 1 <= size_y
            v = y * size_x + x;
            adj_mat(v, u) = ~(matrix(x, y + 1) || matrix(x, y));
         end
         if y - 1 > 0
            v = (y - 2) * size_x + x;
            adj_mat(v, u) = ~(matrix(x, y - 1) || matrix(x, y));
         end
      end
   end

   G = graph(adj_mat);
end
