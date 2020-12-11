classdef Hyperplane < handle
    properties               
        p_;    % point on the plane
        n_;    % Normal of the plane, directional,
        local_bbox_; 
    end
    methods                 
        function obj = Hyperplane(p, n)
            obj.p_ = p;
            obj.n_ = n;
        end
        % Calculate the signed distance from point
        function dis = signed_dist(obj, pt)
            dis = dot(obj.n_, pt - obj.p_);
        end
        % Calculate the distance from point
        function dis = dist(obj, pt)
            dis = abs(obj.signed_dist(pt));
        end
    end
end