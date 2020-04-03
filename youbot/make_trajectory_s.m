function [position, speed, acceleration] = make_trajectory_s(s0, sf, v, k, alpha, ts)
    v = sign(sf - s0) * v;
    a = (k * (alpha + 1) + (1 - alpha)) * v^2 / (k * (sf - s0));
    j = (k - 1) * a^2 * v / (-(k + 1) * v^2 + k * a * (sf - s0));

    T1 = a / j;
    T2 = v / a - a / j;
    T3 = T2 / k;

    T = 4 * T1 + 2 * T2 + T3;
    step = round(T / ts);

    time = linspace(0, (step + 1) * ts, step + 2);
    jerk = zeros(size(time));

    for n = 1 : step + 2
        t = time(n);
        if t >= 0 && t <= T1
            jerk(n) = j;
        elseif t >= T1 + T2 && t <= 2 * T1 + T2
            jerk(n) = -j;
        elseif t >= 2 * T1 + T2 + T3 && t <= 3 * T1 + T2 + T3
            jerk(n) = -j;
        elseif t >= 3 * T1 + 2 * T2 + T3 && t <  4 * T1 + 2 * T2 + T3
            jerk(n) = j;
        end
    end

    acceleration = cumtrapz(time, jerk);
    speed = cumtrapz(time, acceleration);
    position = cumtrapz(time, speed) + s0;

     figure
     hold on
     plot(time, jerk, 'DisplayName', 'jerk', 'linewidth', 1.5);
     plot(time, acceleration, 'DisplayName', 'acceleration', 'linewidth', 1.5);
     plot(time, speed, 'DisplayName', 'speed', 'linewidth', 1.5);
     plot(time, position, 'DisplayName', 'position', 'linewidth', 1.5);
     legend

end
