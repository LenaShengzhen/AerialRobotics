classdef StarConvex < handle
    properties
        % All verticals are in CCW
        xs_;
        ys_;
        zs_;
        vertices_;
        Po_;
        epsilon_ = 1e-5;
        % Output convex region in original space as Ax <= b
        A;
        b;
    end

    properties %(Access = private)
        dim;

        % Constructed convex hull from star convex polytope, 3*n
        Edge_;

        convex_hull;

        % Points inside the constructed convex hull, 3*k
        P_in_;
    end

    methods
        function obj = StarConvex(xs, ys, Po, zs)
            assert(all(size(xs) == size(ys)));
            obj.xs_ = xs;
            obj.ys_ = ys;
            obj.Po_ = Po;
            if nargin > 3
                obj.zs_ = zs;
                obj.dim = 3;
            else
                obj.zs_ = zeros(size(xs));
                obj.dim = 2;
            end
            obj.vertices_ = [obj.xs_; obj.ys_; obj.zs_];
        end

        function ConstructConvexFromStar(obj) 
            % Graham Scan, reference
            % http://www.cosy.sbg.ac.at/~held/teaching/compgeo/cg_study.pdf
            
            % Find the bottommost point
            ymin = obj.vertices_(2, 1);
            min_ind = 1;
            for i = 2:size(obj.vertices_,2)
                y = obj.vertices_(2, i);
                % Pick the bottom-most or choose the left most point in
                % case of tie
                if y < ymin || (y == ymin && obj.vertices_(1, i) < ...
                        obj.vertices_(1, min_ind))
                    ymin = obj.vertices_(2, i);
                    min_ind = i;
                end
            end
            obj.vertices_ = circshift(obj.vertices_, [0, min_ind]);
            
            obj.convex_hull = obj.vertices_;
            obj.P_in_ = [];

            cur_ind = 1;
            init_pt = obj.convex_hull(:, cur_ind);
            done = false;
            while done == false
                if cur_ind == 1
                    pre_ind = size(obj.convex_hull, 2);
                    next_ind = cur_ind + 1;
                elseif cur_ind == size(obj.convex_hull, 2)
                    pre_ind = cur_ind - 1;
                    next_ind = 1;
                else
                    pre_ind = cur_ind - 1;
                    next_ind = cur_ind + 1;
                end
                pre_pt = obj.convex_hull(:, pre_ind);
                cur_pt = obj.convex_hull(:, cur_ind);
                next_pt = obj.convex_hull(:, next_ind);
                cross_turn = cross(cur_pt - pre_pt, next_pt - cur_pt);
%                 fprintf("Pre index is %d, current index is %d, next index is %d \n", pre_ind, cur_ind, next_ind);
%                 fprintf("Pre pt is [%.2f, %.2f, %.2f], cur pt is [%.2f, %.2f, %.2f]," + ...
%                     " next pt is [%.2f, %.2f, %.2f]", pre_pt(1), pre_pt(2), pre_pt(3),...
%                     cur_pt(1), cur_pt(2), cur_pt(3), next_pt(1), next_pt(2), next_pt(3));
%                 fprintf("crossturn is %f \n", cross_turn(3));
                if cross_turn(3) > obj.epsilon_ % left turn if CCW
                    cur_ind = mod(cur_ind, size(obj.convex_hull, 2)) + 1;
                    if norm(obj.convex_hull(:, cur_ind) - init_pt) < obj.epsilon_
                        done = true;
                    end
                elseif cross_turn(3) < -obj.epsilon_ % right turn if CCW
                    obj.P_in_(:, end+1) = obj.convex_hull(:, cur_ind);
                    obj.convex_hull(:, cur_ind) = [];
                    % start from bottom most and left point guarantee 
                    % cur_ind is not 1
                    cur_ind = cur_ind - 1;
                else % collinear
                    obj.convex_hull(:, cur_ind) = [];
                    if norm(obj.convex_hull(:, cur_ind)-init_pt) < obj.epsilon_
                        done = true;
                    end
                end
            end
        end

        function PlotConvexHull(obj)
            plot([obj.convex_hull(1, :), obj.convex_hull(1, 1)], ...
                [obj.convex_hull(2, :), obj.convex_hull(2, 1)], 'k*--');
        end

        function PlotVertices(obj)
            plot([obj.vertices_(1, :), obj.vertices_(1, 1)], ...
                [obj.vertices_(2, :), obj.vertices_(2, 1)], 'b*-');
        end

        function PlotInterPoint(obj)
            plot(obj.P_in_(1, :), obj.P_in_(2, :), 'ro');
        end

        function VisualizeResult(obj)
            figure;
            hold on
            obj.PlotVertices();
            obj.PlotConvexHull();
            obj.PlotInterPoint();
            hold off;
        end
    end

end
