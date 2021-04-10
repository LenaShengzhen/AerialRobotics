classdef Ellipsoid < handle
    properties               
        C_;     % Record the matrix of the ellipse, the semi-axis length of the ellipse without square
        d_;     % Center point of ellipse
        axes_;  % Length of each axis of ellipse
        invC_;  % inv(obj.C_)
    end
    methods                   
        function obj = Ellipsoid (a,b)   
            obj.d_ = b;
            obj.setC(a);
        end
        
        function setC(obj, a)
            obj.C_ = a;
            obj.invC_ = inv(a);
            if isnan(obj.invC_), assert(0 >= 42); end
            if (obj.invC_ == inf), assert(0 >= 42); end
        end

        % Calculate distance to the center
        function dis = dist(obj, pt)
            dis = sqrt(obj.square_of_dist(pt));
        end
        
        % Calculate the square of the distance to the center
        function dis = square_of_dist(obj, pt)
            dd = obj.invC_ * (pt - obj.d_)';
            dis = dot(dd, dd);   % dot(A,B)
        end

        % Check if the point is inside
        function ret = inside(obj, pt)
            ret = false;
            if (obj.square_of_dist(pt) <= 1)
                ret = true;
            end
        end

        % Calculate points inside ellipsoid
        function pinside = points_inside(obj, points)
            pinside = [];
            [len, ~]=size(points);
            for i = 1 : len
                if(obj.inside(points(i,:)))
                    pinside = [pinside; points(i,:)];
                end
            end
        end

        % Find the closest point
        function p = closest_point(obj, points)
            p = points(1,:);
            min_dist = 1e8;
            [len, ~]=size(points);
            for i = 1 : len
                d = obj.dist(points(i,:));
                if(d < min_dist)
                    min_dist = d;
                    p = points(i,:);
                end
            end						
        end

        % Find the closest hyperplane from the closest point, 
        function plane = closest_hyperplane(obj, points)
            closest_pt = obj.closest_point(points);
            n = ( obj.invC_ * (obj.invC_)' )*(closest_pt - obj.d_)';
            plane = Hyperplane(closest_pt, n);
        end
    end
end
