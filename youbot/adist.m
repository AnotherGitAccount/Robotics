function dist = adist(a, b)
    tmp_a = mod(a, 2 * pi);
    tmp_b = mod(b, 2* pi);
    dist = min(abs(tmp_a - tmp_b), 2 * pi - abs(tmp_a - tmp_b));
end

