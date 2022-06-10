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
                obj.vertices_ = [obj.xs_, obj.ys_, obj.zs_];
            else
                obj.dim = 2;
                obj.vertices_ = [obj.xs_, obj.ys_];
            end
        end

        function ConstructConvexFromStar(obj)
            if obj.dim == 2
%                 % Graham Scan, reference
%                 % http://www.cosy.sbg.ac.at/~held/teaching/compgeo/cg_study.pdf
%                 
%                 % Find the bottommost point
%                 ymin = obj.vertices_(1, 2);
%                 min_ind = 1;
%                 for i = 2:size(obj.vertices_,1)
%                     y = obj.vertices_(i, 2);
%                     % Pick the bottom-most or choose the left most point in
%                     % case of tie
%                     if y < ymin || (y == ymin && obj.vertices_(i, 1) < ...
%                             obj.vertices_(min_ind, 1))
%                         ymin = obj.vertices_(i, 2);
%                         min_ind = i;
%                     end
%                 end
%                 obj.vertices_ = circshift(obj.vertices_, [min_ind, 0]);
%                 
%                 obj.convex_hull = obj.vertices_;
%     
%                 cur_ind = 1;
%                 obj.P_in_ = [];
%                 init_pt = obj.convex_hull(cur_ind, :);
%                 done = false;
%                 while done == false
%                     if cur_ind == 1
%                         pre_ind = size(obj.convex_hull, 1);
%                         next_ind = cur_ind + 1;
%                     elseif cur_ind == size(obj.convex_hull, 1)
%                         pre_ind = cur_ind - 1;
%                         next_ind = 1;
%                     else
%                         pre_ind = cur_ind - 1;
%                         next_ind = cur_ind + 1;
%                     end
%                     pre_pt = obj.convex_hull(pre_ind, :);
%                     cur_pt = obj.convex_hull(cur_ind, :);
%                     next_pt = obj.convex_hull(next_ind, :);
%                     v1 = cur_pt - pre_pt;
%                     v2 = next_pt - cur_pt;
%                     cross_turn = v1(1)*v2(2) - v1(2)*v2(1);
%     %                 fprintf("Pre index is %d, current index is %d, next index is %d \n", pre_ind, cur_ind, next_ind);
%     %                 fprintf("Pre pt is [%.2f, %.2f, %.2f], cur pt is [%.2f, %.2f, %.2f]," + ...
%     %                     " next pt is [%.2f, %.2f, %.2f]", pre_pt(1), pre_pt(2), pre_pt(3),...
%     %                     cur_pt(1), cur_pt(2), cur_pt(3), next_pt(1), next_pt(2), next_pt(3));
%     %                 fprintf("crossturn is %f \n", cross_turn(3));
%                     if cross_turn > obj.epsilon_ % left turn if CCW
%                         cur_ind = mod(cur_ind, size(obj.convex_hull, 1)) + 1;
%                         if norm(obj.convex_hull(cur_ind, :) - init_pt) < obj.epsilon_
%                             done = true;
%                         end
%                     elseif cross_turn < -obj.epsilon_ % right turn if CCW
%                         obj.P_in_(end+1, :) = obj.convex_hull(cur_ind, :);
%                         obj.convex_hull(cur_ind, :) = [];
%                         % start from bottom most and left point guarantee 
%                         % cur_ind is not 1
%                         cur_ind = cur_ind - 1;
%                     else % collinear
%                         obj.convex_hull(cur_ind, :) = [];
%                         if norm(obj.convex_hull(cur_ind, :)-init_pt) < obj.epsilon_
%                             done = true;
%                         end
%                     end
%                 end
%                 obj.convex_hull(end+1, :) = obj.convex_hull(1, :);
                k = convhull(obj.vertices_, 'Simplify', true);
                ind_in = setdiff(1:size(obj.vertices_, 1), k);
                obj.P_in_ = obj.vertices_(ind_in, :);
                obj.convex_hull = obj.vertices_(k, :);
                obj.Edges = repmat(struct, 1, size(obj.convex_hull, 1)-1);
                for i = 1: size(obj.convex_hull, 1)-1
                    obj.Edges(i).p1 = obj.convex_hull(i, :);
                    obj.Edges(i).p2 = obj.convex_hull(i+1, :);
                    vec = obj.Edges(i).p2 - obj.Edges(i).p1;
                    n_vec = [-vec(2), vec(1)]; % Left is the inside part
                    obj.Edges(i).n = n_vec / norm(n_vec);
                end
            else
                k = convhull(obj.vertices_, 'Simplify',true);
                ind_in = setdiff(1:size(obj.vertices_, 1), unique(k));
                obj.P_in_ = obj.vertices_(ind_in, :);
                obj.convex_hull = repmat(struct, 1, size(k, 1));
                for i = 1: size(k, 1)
                    obj.convex_hull(i).p1 = obj.vertices_(k(i, 1), :);
                    obj.convex_hull(i).p2 = obj.vertices_(k(i, 2), :);
                    obj.convex_hull(i).p3 = obj.vertices_(k(i, 3), :);
                    % Normal vector of this hyperedge. It points inside if
                    % facet vertices detemined by k are CCW
                    n_vec = cross(obj.convex_hull(i).p3 - obj.convex_hull(i).p1, ...
                        obj.convex_hull(i).p2 - obj.convex_hull(i).p1);
                    % Inverse if the vector point to the other side
                    if dot(n_vec, obj.Po_ - obj.convex_hull(i).p1) < -obj.epsilon_
                        n_vec = -n_vec;
                    end
                    obj.convex_hull(i).n = n_vec / norm(n_vec);
                end
                obj.Edges = obj.convex_hull;
            end
        end

        function ShrinkToConvex(obj)
            % Shrink the convex hull to pointcloud-free convex region
            obj.A = zeros(size(obj.Edges, 2), obj.dim);
            obj.b = zeros(size(obj.Edges, 2), 1);
            for i = 1: size(obj.Edges, 2)
                polyhedron = Polyhedron_();
                base = Hyperplane(obj.Edges(i).p1,...
                    obj.Edges(i).n);
                polyhedron.add(base);
                if obj.dim == 2 % 2d dimension
                    % Add first edge
                    vec1 = obj.Edges(i).p1 - obj.Po_;
                    n_vec1 = [-vec1(2), vec1(1)]; % Left is the inside part
                    n1 = n_vec1 / norm(n_vec1);
                    polyhedron.add(Hyperplane(obj.Po_, n1));
                    % Add second edge
                    vec2 = obj.Po_ - obj.Edges(i).p2;
                    n_vec2 = [-vec2(2), vec2(1)];
                    n2 = n_vec2 / norm(n_vec2);
                    polyhedron.add(Hyperplane(obj.Po_, n2));
                else % 3d dimension
                    % Add first edge
                    n1 = cross(obj.Edges(i).p2 - obj.Po_, ...
                        obj.Edges(i).p1 - obj.Po_);
                    n1 = n1 / norm(n1);
                    if dot(n1, obj.Edges(i).p3 - obj.Po_) < -obj.epsilon_
                        n1 = -n1;
                    end
                    polyhedron.add(Hyperplane(obj.Po_, n1));
                    % Add second edge
                    n2 = cross(obj.Edges(i).p3 - obj.Po_, ...
                        obj.Edges(i).p2 - obj.Po_);
                    n2 = n2 / norm(n2);
                    if dot(n2, obj.Edges(i).p1 - obj.Po_) < -obj.epsilon_
                        n2 = -n2;
                    end
                    polyhedron.add(Hyperplane(obj.Po_, n2));
                    % Add third edge
                    n3 = cross(obj.Edges(i).p1 - obj.Po_, ...
                        obj.Edges(i).p3 - obj.Po_);
                    n3 = n3 / norm(n3);
                    if dot(n3, obj.Edges(i).p2 - obj.Po_) < -obj.epsilon_
                        n3 = -n3;
                    end
                    polyhedron.add(Hyperplane(obj.Po_, n3));
                end
                % Iterate every point inside, find the furthest to base
                furthest_dis = -1;
                p_i = obj.Edges(i).p1;
                for j = 1: size(obj.P_in_, 1)
                    p_in = obj.P_in_(j,:);
                    if (polyhedron.inside(p_in)) % CHECK IF IT NEEDS TO BE REVERSED
                        dis = base.signed_dist(p_in);
                        if (dis >= furthest_dis)
                            p_i = p_in;
                            furthest_dis = dis;
                        end
                    end
                end
                % The normal vector in the paper is pointing to outside.
                obj.A(i, :) = -base.n_;
                obj.b(i) = dot(-base.n_, p_i);
%                 if i == 12
%                     ;
%                 end
%                 disp("Current point Po is: ");
%                 disp(obj.Po_);
%                 if dot(obj.A(i, :), obj.Po_) >= obj.b(i)
%                     disp("Shrink the edge inside the pos.");
%                 end
                assert(dot(obj.A(i, :), obj.Po_) < obj.b(i), "Shrink the edge inside the pos.");
            end
        end

        function PlotConvexHull(obj)
            plot(obj.convex_hull(:, 1), obj.convex_hull(:, 2), 'k*--');
        end

        function PlotVertices(obj)
            plot([obj.vertices_(:, 1); obj.vertices_(1, 1)], ...
                [obj.vertices_(:, 2); obj.vertices_(1, 2)], 'b*-');
        end

        function PlotInterPoint(obj)
            plot(obj.P_in_(:, 1), obj.P_in_(:, 2), 'ro');
        end

        function PlotResults(obj)
            upper = max(obj.vertices_);
            lower = min(obj.vertices_);
            range = upper - lower;
            upper = upper + abs(range * 0.1);
            lower = lower - abs(range * 0.1);
            if obj.dim == 2
                [xx, yy] = meshgrid(linspace(lower(1), upper(1), 13),...
                    linspace(lower(2), upper(2), 13));
                region = ones(size(xx));
                for i=1:size(obj.b, 1)
                    sub_region = obj.A(i, 1).*xx + obj.A(i, 2).*yy <= obj.b(i);
                    region = region & sub_region;
                end
                surf(xx,yy,double(region));
                colorbar;
                view(0,90);
            else
                [xx, yy, zz] = meshgrid(linspace(lower(1), upper(1), 13),...
                    linspace(lower(2), upper(2), 13),...
                    linspace(lower(3), upper(3), 13));
                region = ones(size(xx));
                for i=1:size(obj.b, 1)
                    sub_region = obj.A(i, 1).*xx + obj.A(i, 2).*yy +...
                        obj.A(i, 3).*zz <= obj.b(i);
                    region = region & sub_region;
                end

                scatter3(xx(region), yy(region), zz(region));
%                 p=patch(isosurface(region, 0));
%                 set(p,'FaceColor','red','EdgeColor','none');
                daspect([1,1,1])
                view(3); axis equal
                camlight
                lighting gouraud
                grid on
            end
        end

        function VisualizeResult(obj)
            figure;
            hold on
            if obj.dim == 2
                obj.PlotVertices();
                obj.PlotConvexHull();
                obj.PlotInterPoint();
            end
            obj.PlotResults();
            hold off;
        end
    end

end
