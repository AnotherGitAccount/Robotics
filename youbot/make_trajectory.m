function [s, sp] = make_trajectory(s0, sf, v, k, ts)
    v = sign(sf - s0) * v;
    a = v^2 / (k * (sf - s0));

    T1 = v / a;
    T2 = (sf - s0) / v;
    T3 = T1 + T2;

    step = round(T3 / ts);
    time = linspace(0, (step + 1) * ts, step + 2);

    s = zeros(size(time));
    sp = zeros(size(time));

    for n = 1 : step + 2
        ti = time(n);

        if ti <= T1
            s(n) = 1/2 * a * ti^2 + s0;
            sp(n) = a * ti;
        elseif ti > T1 && ti <= T2
            s(n) = v * (ti - T1) + 1/2 * a * T1^2 + s0;
            sp(n) = v;
        elseif ti > T2 && ti <= T3
            s(n) = v * (ti - T1) - 1/2 * a * ((ti - T2)^2 - T1^2) + s0;
            sp(n) = v - a * (ti - T2);
        else
           s(n) = sf;
           sp(n) = 0;
        end
    end
end
