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

        % Constructed convex hull from star convex polytope, 2/3*n
        convex_hull;
        
        % Hyperedges
        Edges;

        % Points inside the constructed convex hull, 2/3*k
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
            obj.vertices_ = [obj.xs_, obj.ys_, obj.zs_];
        end

        function ConstructConvexFromStar(obj)
            if obj.dim == 2
                % Graham Scan, reference
                % http://www.cosy.sbg.ac.at/~held/teaching/compgeo/cg_study.pdf
                
                % Find the bottommost point
                ymin = obj.vertices_(1, 2);
                min_ind = 1;
                for i = 2:size(obj.vertices_,1)
                    y = obj.vertices_(i, 2);
                    % Pick the bottom-most or choose the left most point in
                    % case of tie
                    if y < ymin || (y == ymin && obj.vertices_(i, 1) < ...
                            obj.vertices_(min_ind, 1))
                        ymin = obj.vertices_(i, 2);
                        min_ind = i;
                    end
                end
                obj.vertices_ = circshift(obj.vertices_, [min_ind, 0]);
                
                obj.convex_hull = obj.vertices_;
    
                cur_ind = 1;
                obj.P_in_ = [];
                init_pt = obj.convex_hull(cur_ind, :);
                done = false;
                while done == false
                    if cur_ind == 1
                        pre_ind = size(obj.convex_hull, 1);
                        next_ind = cur_ind + 1;
                    elseif cur_ind == size(obj.convex_hull, 1)
                        pre_ind = cur_ind - 1;
                        next_ind = 1;
                    else
                        pre_ind = cur_ind - 1;
                        next_ind = cur_ind + 1;
                    end
                    pre_pt = obj.convex_hull(pre_ind, :);
                    cur_pt = obj.convex_hull(cur_ind, :);
                    next_pt = obj.convex_hull(next_ind, :);
                    cross_turn = cross(cur_pt - pre_pt, next_pt - cur_pt);
    %                 fprintf("Pre index is %d, current index is %d, next index is %d \n", pre_ind, cur_ind, next_ind);
    %                 fprintf("Pre pt is [%.2f, %.2f, %.2f], cur pt is [%.2f, %.2f, %.2f]," + ...
    %                     " next pt is [%.2f, %.2f, %.2f]", pre_pt(1), pre_pt(2), pre_pt(3),...
    %                     cur_pt(1), cur_pt(2), cur_pt(3), next_pt(1), next_pt(2), next_pt(3));
    %                 fprintf("crossturn is %f \n", cross_turn(3));
                    if cross_turn(3) > obj.epsilon_ % left turn if CCW
                        cur_ind = mod(cur_ind, size(obj.convex_hull, 1)) + 1;
                        if norm(obj.convex_hull(cur_ind, :) - init_pt) < obj.epsilon_
                            done = true;
                        end
                    elseif cross_turn(3) < -obj.epsilon_ % right turn if CCW
                        obj.P_in_(end+1, :) = obj.convex_hull(cur_ind, :);
                        obj.convex_hull(cur_ind, :) = [];
                        % start from bottom most and left point guarantee 
                        % cur_ind is not 1
                        cur_ind = cur_ind - 1;
                    else % collinear
                        obj.convex_hull(cur_ind, :) = [];
                        if norm(obj.convex_hull(cur_ind, :)-init_pt) < obj.epsilon_
                            done = true;
                        end
                    end
                end
                obj.convex_hull(end+1, :) = obj.convex_hull(1, :);
                
                obj.Edges = repmat(struct, 1, size(obj.convex_hull, 1)-1);
                for i = 1: size(obj.convex_hull, 1)-1
                    obj.Edges(i).p1 = obj.convex_hull(i, :);
                    obj.Edges(i).p2 = obj.convex_hull(i+1, :);
                    vec = obj.Edges(i).p2 - obj.Edges(i).p1;
                    n_vec = [-vec(2), vec(1)];
                    obj.Edges(i).n = n_vec / norm(n_vec);
                end
            end

            k = convhull(obj.vertices_, 'Simplify',true);
            ind_in = setdiff(1:size(obj.vertices_, 1), reshape(k, 1, 3 * size(k, 1)));
            obj.P_in_ = obj.vertices_(ind_in, :);
            obj.convex_hull = repmat(struct, 1, size(k, 1));
            for i = 1: size(k, 1)
                obj.convex_hull(i).p1 = obj.vertices_(k(i, 1), :);
                obj.convex_hull(i).p2 = obj.vertices_(k(i, 2), :);
                obj.convex_hull(i).p3 = obj.vertices_(k(i, 3), :);
                %%%%%% TODO implement normal vector of this hyperedge.
            end

        end

%         function ShrinkToConvex(obj)
%             
%         end

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
