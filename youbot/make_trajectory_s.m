function [position, speed, acceleration] = make_trajectory_s(s0, sf, v, k, alpha, step)
    a = (k * (alpha + 1) + (1 - alpha)) * v^2 / (k * (sf - s0));
    j = (k - 1) * a^2 * v / (-(k + 1) * v^2 + k * a * (sf - s0));
    
    T1 = a / j;
    T2 = v / a - a / j;
    T3 = T2 / k;
    
    time = linspace(0, 4 * T1 + 2 * T2 + T3, step);
    jerk = zeros(size(time));
    
    for n = 1 : step
        t = time(n);
        if t >= 0 && t <= T1
            jerk(n) = j;
        elseif t >= T1 + T2 && t <= 2 * T1 + T2
            jerk(n) = -j;
        elseif t >= 2 * T1 + T2 + T3 && t <= 3 * T1 + T2 + T3
            jerk(n) = -j;
        elseif t >= 3 * T1 + 2 * T2 + T3 && t <=  4 * T1 + 2 * T2 + T3
            jerk(n) = j;
        end
    end
    
    dt = (4 * T1 + 2 * T2 + T3) / (step - 1);
    acceleration = zeros(size(time));
    speed = zeros(size(time));
    position = zeros(size(time));
    
    acceleration(1) = 0;
    speed(1) = 0;
    position(1) = s0;
    
    for n = 2:step
        acceleration(n) = acceleration(n - 1) + (jerk(n - 1) + jerk(n)) * dt/2;
        speed(n) = speed(n - 1) + (acceleration(n - 1) + acceleration(n)) * dt/2;
        position(n) = position(n - 1) + (speed(n - 1) + speed(n)) * dt/2;
    end

%     fprintf("a %f - j %f - T1 %f - T2 %f - T3 %f\n", a, j, T1, T2, T3);
%     
%     figure
%     hold on
%     plot(time, jerk, "DisplayName", "jerk", "linewidth", 1.5);
%     plot(time, acceleration, "DisplayName", "acceleration", "linewidth", 1.5);
%     plot(time, speed, "DisplayName", "speed", "linewidth", 1.5);
%     plot(time, position, "DisplayName", "position", "linewidth", 1.5);
%     legend
end

