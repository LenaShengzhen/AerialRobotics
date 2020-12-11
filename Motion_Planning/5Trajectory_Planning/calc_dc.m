% Calculation Derivative coefficient
% n_order = Polynomial equation maximum order
% d = derivertive order =  0:pos  1:vel  2:acc 3:jerk
function t_derC = calc_dc(t, n_order, d)
    t_derC = zeros(1,n_order+1);
    for i = d+1 : n_order+1
        t_derC(i) = prod(i-d:i-1) * t^(i-d-1);
    end
end
