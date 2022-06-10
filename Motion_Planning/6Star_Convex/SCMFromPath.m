function [A, b] = SCMFromPath(path, point_cloud_obs, map_boundary)
% LargeConvexPolytopes generates A and b from a single point, SCMFrom path
% generate A and b from a path.
    map_size = min(abs(map_boundary.ld - map_boundary.ru));
%     fprintf("Map size is: %f \n", map_size);
    R = map_size/2;
    tolerance = 0.01;
    A = {};
    b = {};
    ieq = 1;
    n_node = size(path, 1) - 1;
    pos = path(1, :);
    [cur_A, cur_b] = LargeConvexPolytopes(point_cloud_obs, pos, R);
    A{1} = cur_A;
%     cur_b = cur_A * pos' + cur_b;
    b{1} = cur_b;
    for i = 1:n_node
%         fprintf("Node number %d \n", i);
%         if i == 7
%             ;
%         end
        cur_pt = path(i, :);
        next_pt = path(i+1, :);
%         disp(cur_pt);
%         disp(next_pt);
        while any(cur_A * next_pt' >= cur_b)
            if any(cur_A * cur_pt' >= cur_b)
                disp("Wrong! Current point is not in the polytope!");
            end
%             fprintf("Checking node number %d \n", i);
            start = 0;
            goal = 1;
            distance = norm(next_pt - cur_pt);
%             fprintf("Distance between start and goal is %f .\n", distance);
            while distance*(goal - start) > tolerance
                mid = start + (goal - start) / 2;
                mid_pt = cur_pt + (next_pt - cur_pt) * mid;
%                 fprintf("Start is %f, middle is %f, goal is %f.\n", start, mid, goal);
                if any(cur_A * mid_pt' >= cur_b)
                    goal = mid;
                else
                    start = mid;
                end
            end
            cur_pt = cur_pt + (next_pt - cur_pt) * goal;
            [cur_A, cur_b] = LargeConvexPolytopes(point_cloud_obs, cur_pt, R);
            if any(cur_A * cur_pt' >= cur_b)
                disp("Wrong! Current point is not in the polytope!");
            end
            A{ieq+1} = cur_A;
%             cur_b = cur_A * cur_pt' + cur_b;
            b{ieq+1} = cur_b;
            ieq = ieq + 1;
        end
    end
end