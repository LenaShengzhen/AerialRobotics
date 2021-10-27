classdef HyperEdge < handle
    properties
        % In 2d, it has 2 points as an edge; in 3d, it has 3 points as a facet.
        p1_;
        p2_;
        p3_;
        n_;
    end

    methods
        function obj = HyperEdge(p1, p2, p3)
            obj.p1_ = p1;
            obj.p2_ = p2;
            if argmin > 2
                obj.p3_ = p3;
            end
            
        end
    end
    
end