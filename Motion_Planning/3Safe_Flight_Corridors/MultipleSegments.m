classdef MultipleSegments < handle
    properties
        lines_;
        obs_;
        local_bbox_;
    end
    methods 
        function obj = MultipleSegments(local_bbox)   
            obj.local_bbox_ = local_bbox;
        end

        function set_obs(obj, obs)
            obj.obs_ = obs;
        end

        function dilate(obj, path, offset_x)
            [len, ~] = size(path);
            obj.lines_ = cell(1, len-1);

            for i = 1 : (len - 1)
                p1 = path(i,:); p2 = path(i+1,:);
                obj.lines_{i} = LineSegment(p1, p2);
                obj.lines_{i}.set_local_bbox(obj.local_bbox_);
                obj.lines_{i}.set_obs(obj.obs_);
                obj.lines_{i}.dilate(offset_x);
            end	
            
        end	
        
        function drawLines(obj)
            len = length(obj.lines_);
            for i = 1 : len	
                obj.lines_{i}.p1_;
            end
        end
        
        function drawEllipsoids(obj)
            hold on;
            len = length(obj.lines_);
            for i = 1 : len	
                center = obj.lines_{i}.ellipsoid_.d_;
                R = obj.lines_{i}.R_;
                axes = obj.lines_{i}.ellipsoid_.axes_;                
                obj.draw_Ellipsoid(center, axes, R');
            end
            hold off;
        end
        
        function draw_Ellipsoid(obj, center, axes, R)
            [X, Y, Z] = ellipsoid(0,0,0, axes(1), axes(2), axes(3));
            [row, col] = size(X);
            A = [X(:), Y(:), Z(:)];
            B = A*R + center;
            X = reshape(B(:,1),row,col);
            Y = reshape(B(:,2),row,col);
            Z = reshape(B(:,3),row,col);
            mesh(X, Y, Z, 'FaceAlpha','0.5');
        end
        
        % draw the point of obs on the polyhedron 
        function drawPlanePoint(obj)
            hold on;
            len = length(obj.lines_);
            planesCell = obj.lines_{len}.polyhedron_.polys_;
            [~, lenj] = size(planesCell);
            for i = 1 : lenj	
                point = planesCell{i}.p_;
                plot3(point(1), point(2), point(3),'r*');
            end
            hold off;
        end
        
        function drawpolyhedron(obj)
            hold on;
            len = length(obj.lines_);
            for i = len : len
                planesCell = obj.lines_{i}.polyhedron_.polys_;
                p1 = obj.lines_{i}.p1_;
                p2 = obj.lines_{i}.p2_;
                r = norm(p2 - p1)/2;
                pmid = (p2 + p1)/2;
                [~, lenj] = size(planesCell);
                for j = 1 : lenj
                    obj.drawPlane(planesCell{j}.n_, planesCell{j}.p_, pmid, r);
                end
            end
            hold off;
        end
        
        function drawPlane(obj, n, p, pmid, r)
            r = r*2;
            xL = pmid(1) - r;
            xR = pmid(1) + r;
            yU = pmid(2) + r;
            yD = pmid(2) - r;
            
            [X,Y]=meshgrid(xL:0.5:xR, yD:0.5:yU);
            Z = p(3) - ((X - p(1))*n(1)+(Y - p(2))*n(2))/n(3);
            surf(X, Y, Z, 'FaceAlpha','0.5','EdgeColor','flat','FaceColor', '[0 1 1]');
        end
    end
end