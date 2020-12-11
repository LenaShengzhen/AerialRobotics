%   averageSpeed_TimeAllocation:
%   avg_v: the estimated average speed
function [ts, total_time] = averageSpeed_ta(path, avg_v)
    path_seg_len = sqrt(sum((path(2:end, :) - path(1:end-1,:)).^2,2));
    path_len = sum(path_seg_len);
    total_time = path_len/avg_v;

    ts = cumsum(path_seg_len);
    ts = ts/ts(end);
    ts = [0; ts]';
    ts = ts*total_time;
end


