function loc = getLocations(x, y, a)
   % a = number of +90° rotations
   loc = zeros(2, 10);
   if a == 0
      loc(:, 1)  = [x + 1; y + 1];
      loc(:, 2)  = [x    ; y + 1];
      loc(:, 3)  = [x - 1; y + 1];
      loc(:, 4)  = [x + 1; y];
      loc(:, 5)  = [x    ; y];
      loc(:, 6)  = [x - 1; y];
      loc(:, 7)  = [x + 1; y - 1];
      loc(:, 8)  = [x    ; y - 1];
      loc(:, 9)  = [x - 1; y - 1];
      %loc(:, 10) = [x    ; y - 2];
   elseif a == 1
      loc(:, 1)  = [x - 1; y + 1];
      loc(:, 2)  = [x - 1; y];
      loc(:, 3)  = [x - 1; y - 1];
      loc(:, 4)  = [x    ; y + 1];
      loc(:, 5)  = [x    ; y];
      loc(:, 6)  = [x    ; y - 1];
      loc(:, 7)  = [x + 1; y + 1];
      loc(:, 8)  = [x + 1; y];
      loc(:, 9)  = [x + 1; y - 1];
      %loc(:, 10) = [x + 2; y];
   elseif a == 2
      loc(:, 1)  = [x - 1; y - 1];
      loc(:, 2)  = [x    ; y - 1];
      loc(:, 3)  = [x + 1; y - 1];
      loc(:, 4)  = [x - 1; y];
      loc(:, 5)  = [x    ; y];
      loc(:, 6)  = [x + 1; y];
      loc(:, 7)  = [x - 1; y + 1];
      loc(:, 8)  = [x    ; y + 1];
      loc(:, 9)  = [x + 1; y + 1];
      %loc(:, 10) = [x    ; y + 2];
   elseif a == 3
      loc(:, 1)  = [x + 1; y - 1];
      loc(:, 2)  = [x + 1; y];
      loc(:, 3)  = [x + 1; y + 1];
      loc(:, 4)  = [x    ; y - 1];
      loc(:, 5)  = [x    ; y];
      loc(:, 6)  = [x    ; y + 1];
      loc(:, 7)  = [x - 1; y - 1];
      loc(:, 8)  = [x - 1; y];
      loc(:, 9)  = [x - 1; y + 1];
      %loc(:, 10) = [x - 2; y];
   else
      error('Unknow value for a - must be in [0, 3]');
   end
end
