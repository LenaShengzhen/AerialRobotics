%   trapezoidalSpeed_TimeAllocation:
%   avg_v: the estimated average speed
%   acc: Acceleration  trapezoidalSpeed
function [ts, total_time] = trapezoidalSpeed_ta(path, avg_v, acc)

    t_acc = avg_v/acc;
    s_acc = 0.5*acc*(t_acc^2);
    
    path_seg_len = sqrt(sum((path(2:end, :) - path(1:end-1,:)).^2,2));
    path_len = sum(path_seg_len);
    total_time = path_len/avg_v;
    
    ts = cumsum(path_seg_len);
    ts = ts/ts(end);
    ts = [0; ts]';
    ts = ts*total_time;
    
    
    % trapezoid Area = 0.5(a + b) × h
    % math ep
%     a1 = ts(2) - t_acc;
%     b1 = a1 - speed_h/acc;
%     ep1 = 0.5*(a1 + b1)*speed_h + t_add1*(avg_v + speed_h) == s_acc;
%     a2 = ts(end) - ts(end-1) - t_acc;
%     b2 = a2 - speed_h/acc;
%     ep2 = 0.5*(a2 + b2)*speed_h + t_add2*(avg_v + speed_h) == s_acc;
%     t_mid = (ts(end-1) - ts(2));
%     ep3 = (t_mid-t_add1-t_add2)*speed_h - (t_add1 + t_add2)*avg_v == 0;
     

    %% solve
    syms t_add1 t_add2 speed_h;
    a1 = ts(2) - t_acc;
    ep1 = 0.5*(a1 + a1 - speed_h/acc)*speed_h + t_add1*(avg_v + speed_h) == s_acc;
    a2 = ts(end) - ts(end-1) - t_acc;
    ep2 = 0.5*(a2 + a2 - speed_h/acc)*speed_h + t_add2*(avg_v + speed_h) == s_acc;
    t_mid = (ts(end-1) - ts(2));
    ep3 = (t_mid-t_add1-t_add2)*speed_h - (t_add1 + t_add2)*avg_v == 0;
    [x0,y0,z0] = solve(ep1,ep2,ep3, t_add1, t_add2, speed_h);
    ans = double([x0,y0,z0]);
    
    sort_ans = sortrows(ans,3);
    
    ts(2) = ts(2) + sort_ans(1,1);
    ts(end-1) = ts(end-1) - sort_ans(1,2); 
    
    if(path_len < 2*s_acc), assert(1 >= 2); end
    if(path_seg_len(1)   < s_acc), assert(1 >= 2); end
    if(path_seg_len(end) < s_acc), assert(1 >= 2); end
    
end

