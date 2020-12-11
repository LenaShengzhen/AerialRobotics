classdef GridMap < handle
    properties
        boundary;       % size of Metric Map(leftdownPoint + rightupPoint)
        blocks;         % coordinate of obstacles on the Metric Map
        res;            % resolution
        margin;
        maxIndex;       % max index of ijk of GridMap
        occ_map;        % flag obstacles on the GridMap
    end
    methods 
        function obj = GridMap()
        end
        
        function load_map(obj, filename, xy_res, z_res, margin)
            fid = fopen(filename);
            tline = fgets(fid);
            obj.blocks = [];
            while ischar(tline)
                % skip empty line and comments
                if numel(tline) > 1 & tline(1)~='#'
                    % convert char array to string
                    if strncmpi(tline, 'boundary', 8)
                        boundarys = strread(tline(9:end));
                    end
                    if strncmpi(tline, 'block', 5)
                        block = strread(tline(6:end));
                        assert(size(block,2) == 9);
                        obj.blocks = [obj.blocks; block];
                    end
                end
                tline = fgets(fid);
            end
            obj.boundary.ld = boundarys(1:3);
            obj.boundary.ru = boundarys(4:6);
            obj.res = [xy_res xy_res z_res];
            obj.margin = margin;
            obj.maxIndex = ceil(abs( (obj.boundary.ru - obj.boundary.ld)./obj.res ));            
        end
        
        % obstacles = block + margin
        function flag_obstacles(obj)
            obj.occ_map = false(obj.maxIndex);    
            for i = 1 : size(obj.blocks,1)
                block = obj.blocks(i, :);
                leftdownPoint = [min(block(1), block(4)) min(block(2), block(5)) min(block(3), block(6))];
                rightUpPoint = [max(block(1), block(4)) max(block(2), block(5)) max(block(3), block(6))];
                obj.blocks(i, 1:3) = leftdownPoint;
                obj.blocks(i, 4:6) = rightUpPoint;
                xyzs = obj.points_to_idx(leftdownPoint - obj.margin);
                xyze = obj.points_to_idx(rightUpPoint + obj.margin);
                obj.occ_map(xyzs(1):xyze(1), xyzs(2):xyze(2), xyzs(3):xyze(3)) = 1;
            end
        end
        
        function ijk = points_to_idx(obj, xyz)
            assert(size(xyz,1) > 0);
            ijk = floor((xyz - obj.boundary.ld)./obj.res + 1);
            ijk = min(max(ijk, 1), obj.maxIndex);
        end
        
        function xyz = idx_to_points(obj, ijk)
            assert(size(ijk,1) > 0);
            xyz = obj.boundary.ld + (ijk - 1 + 0.5).* obj.res;
        end
        
        function ret = idOverflowCheck(obj, i, j, k)
            ret = (i < 1 || i > obj.maxIndex(1) || j < 1 || j > obj.maxIndex(2) || k < 1 || k > obj.maxIndex(3));
        end
        
        % check points collide
        function C = collide(obj, points)
            ijk = obj.points_to_idx(points);
            C = false(size(ijk, 1), 1);
            for i = 1 : size(ijk, 1)
                id = ijk(i,:);
                C(i) =  obj.occ_map(id(1), id(2), id(3)) ~= 0;
            end
        end
                
        %% generate intermedia points
        function pts = generateiIntermediaPoints(obj, start_pos, end_pos)
            %% generate Tiny direction vector
            dd = end_pos - start_pos;
            dir = obj.res(1)*(dd/norm(dd, 2))/(100);
            
            %% generate intermedia points
            lenxy = sqrt(sum((start_pos(1:2) - end_pos(1:2)).^2));
            lenz = sqrt(sum((start_pos(3) - end_pos(3)).^2));
            n_pts = int32(max(lenxy/obj.res(1), lenz/obj.res(3))) + 3;
            pts = [linspace(start_pos(1), end_pos(1), n_pts);
                   linspace(start_pos(2), end_pos(2), n_pts);
                   linspace(start_pos(3), end_pos(3), n_pts)]';
               
            %% mid-point + Tiny direction vector   
            for j = 2 : size(pts,1) - 1
                m1 = mod(pts(j,1), obj.res(1));
                m2 = mod(pts(j,2), obj.res(1));
                m3 = mod(pts(j,3), obj.res(1));
                if(m1*m2*m3 == 0)
                    pts(j,:) = pts(j,:) + dir;
                end
            end
        end
    end
end

