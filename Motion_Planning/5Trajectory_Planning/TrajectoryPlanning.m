function [total_time, ts, X] = TrajectoryPlanning(path, decomp, time_allocation)
    speed   = time_allocation.avg_speed;
    acc     = time_allocation.acc;
    
    disp(['The planned speed is : ', num2str(speed)]);
    tic
    if strcmp(time_allocation.type, 'averageSpeed')
        [ts, total_time] = averageSpeed_ta(path0, speed);
    elseif strcmp(time_allocation.type, 'trapzoidSpeed')
        [ts, total_time] = trapezoidalSpeed_ta(path, speed, acc);
    end
    disp('TimeAllocation time is :');
    toc
    disp(['time management: total_time is ', num2str(total_time), 'seconds']);
    disp(['Split time is : ', num2str(ts)]);
    
    
    tic 
    %  use Ax=b get Trajectory planning. ===========================
    %  X = Ax_equal_b(path, total_time, ts);

    % use 'Quadratic Programming' and SFC(which make by Ax < b) get Trajectory planning. ========================
    % use SFC make Inequality constraints
    X = QPbyUseSFC(path, ts, decomp);

    disp('generator trajectory time is :');
    toc
end

