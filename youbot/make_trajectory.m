function [s, sp] = make_trajectory(s0, sf, v, k, steps)
    a = v^2 / (k * (sf - s0));

    T1 = v / a;
    T2 = (sf - s0) / v;
    T3 = T1 + T2;
    
    t = linspace(0, T1 + T2, steps);
    s = 1 : steps;
    sp = 1 : steps;
    
    for n = 1 : steps
        ti = t(n);
        
        if ti <= T1
            s(n) = 1/2 * a * ti^2 + s0;
            sp(n) = a * ti;
        elseif ti > T1 && ti <= T2
            s(n) = v * (ti - T1) + 1/2 * a * T1^2 + s0;
            sp(n) = v;
        elseif ti > T2 && ti <= T3
            s(n) = v * (ti - T1) - 1/2 * a * ((ti - T2)^2 - T1^2) + s0;
            sp(n) = v - a * (ti - T2);
        end
    end
end

