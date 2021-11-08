classdef Polyhedron_ < handle
    properties               
        polys_;    
        epsilon_;  
    end
    methods                 
        function obj = Polyhedron_()   
            obj.polys_ = cell(0);
            obj.epsilon_ = 1e-10;
        end

        % Append Hyperplane
        function add(obj, plane)
            % Does normal vector of the plane point inside or outside?
            obj.polys_{end+1} = plane;
        end
        % Check if the point is inside polyhedron,
        function isinside = inside(obj, pt)
            isinside = false;
            [~, len]=size(obj.polys_);
                for i = 1 : len
                    if(obj.polys_{i}.signed_dist(pt) < -obj.epsilon_)
                        return;
                    end
                end
            isinside = true;
        end

        % Calculate points inside polyhedron
        function pinside = points_inside(obj, points)
            pinside = [];
            [len, ~]=size(points);
            for i = 1 : len
                if(obj.inside(points(i,:)))
                    pinside = [pinside; points(i,:)];
                end
            end
        end
    end
end