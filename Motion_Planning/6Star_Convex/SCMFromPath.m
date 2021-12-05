function [A, b] = SCMFromPath(path, point_cloud_obs, map_boundary)
% LargeConvexPolytopes generates A and b from a single point, SCMFrom path
% generate A and b from a path.
    map_size = min(map_boundary.ld - map_boundary.ru);
    R = map_size;
    tolerance = 0.1;
    A = {};
    b = {};
    ieq = 1;
    n_node = size(path, 1) - 1;
    pos = path(1, :);
    [cur_A, cur_b] = LargeConvexPolytopes(point_cloud_obs, pos, R);
    A{1} = cur_A;
    b{1} = cur_b;
    for i = 1:n_node
        cur_pt = path(i, :);
        next_pt = path(i+1, :);
        while any(cur_A * next_pt' >= cur_b)
            start = 0;
            goal = 1;
            distance = norm(next_pt - cur_pt);
            while distance*(goal - start) > tolerance
                mid = start + (goal - start) / 2;
                mid_pt = cur_pt + (next_pt - cur_pt) * mid;
                if any(cur_A * mid_pt' >= cur_b)
                    goal = mid;
                else
                    start = mid;
                end
            end
            cur_pt = cur_pt + (next_pt - cur_pt) * goal;
            [cur_A, cur_b] = LargeConvexPolytopes(point_cloud_obs, cur_pt, R);
            A{ieq+1} = cur_A;
            b{ieq+1} = cur_b;
            ieq = ieq + 1;
        end
    end
end