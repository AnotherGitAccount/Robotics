function dist = adist(a, b)
   s = 1;
   if ((pi <= b - a && b - a <= 2*pi) || (-pi <= b - a && b - a <= 0))
      s = -1;
   end

   tmp_a = mod(a, 2 * pi);
   tmp_b = mod(b, 2* pi);
   dist = s * min(abs(tmp_a - tmp_b), 2 * pi - abs(tmp_a - tmp_b));
end
