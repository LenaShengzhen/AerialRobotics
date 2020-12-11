% Calculation Q_1 of Q_x of Q of p^TQp 
% n:Polynomial equation maximum order
% d:derivertive order, 1:minimum vel 2:minimum acc 3:minimum jerk 4:minimum snap
% t1:start timestamp for polynormial
% t2:end timestap for polynormial
function Q1 = calc_Q(n_order, d, t1, t2)
    T = zeros((n_order-d)*2+1,1);
    for n = 1:(n_order-d)*2+1
        T(n) = t2^n-t1^n;
    end
    Q1 = zeros(n_order);
    for i = d+1 : n_order+1
        for j = i : n_order+1
            c = i + j - 2*d - 1;
            Q1(i,j) = prod(i-d:i-1) * prod(j-d:j-1) / c * T(c);
            Q1(j,i) = Q1(i,j);
        end
    end

end