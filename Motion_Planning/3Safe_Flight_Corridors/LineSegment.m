classdef LineSegment < handle
    properties               
        p1_;         
        p2_;        
        R_;
        obs_;        
        ellipsoid_;  
        polyhedron_;
        local_bbox_; % Bounding Box of map
        epsilon_;   
    end
    methods                  
        function obj = LineSegment(p1, p2)   
            obj.p1_ = p1;
            obj.p2_ = p2;
            obj.epsilon_ = 1e-10;
        end

        function dilate(obj, radius)
            obj.find_ellipsoid(radius);  
            obj.find_polyhedron();       
            obj.add_local_bbox(obj.polyhedron_); 				
        end

        function set_local_bbox(obj, local_bbox)
            obj.local_bbox_ = local_bbox;				
        end		

        function set_obs(obj, obs)
            Vs = Polyhedron_();
            obj.add_local_bbox(Vs);		
            obj.obs_ = Vs.points_inside(obs);	
        end		

        function add_local_bbox(obj, Vs)
            r = norm(obj.p2_ - obj.p1_)/2;  
            dir = (obj.p2_ - obj.p1_)/norm(obj.p2_ - obj.p1_);
            dir_h = [dir(2) -dir(1) 0];
            if(norm(dir_h) == 0)
                    dir_h = [-1 0 0];  
            end
            dir_h = dir_h/norm(dir_h);

            % along x
            pp1 = obj.p1_ + dir_h*(r);
            pp2 = obj.p1_ - dir_h*(r);
            Vs.add(Hyperplane(pp1, dir_h));
            Vs.add(Hyperplane(pp2, -dir_h));

            % along y		
            pp3 = obj.p2_ + dir*(r);
            pp4 = obj.p1_ - dir*(r);
            Vs.add(Hyperplane(pp3, dir));
            Vs.add(Hyperplane(pp4, -dir));		

            % along z, 
            dir_v = [0 0 0];	
            dir_v(1) =  dir(2) * dir_h(3) - dir(3) * dir_h(2);
            dir_v(2) =  dir(3) * dir_h(1) - dir(1) * dir_h(3); 
            dir_v(3) =  dir(1) * dir_h(2) - dir(2) * dir_h(1); 
            pp5 = obj.p1_ + dir_v*(r);
            pp6 = obj.p1_ - dir_v*(r);
            Vs.add(Hyperplane(pp5, dir_v));
            Vs.add(Hyperplane(pp6, -dir_v));
        end

        % 3D 
        function find_ellipsoid(obj, offset_x)
            f = norm(obj.p1_ - obj.p2_)/2;  
            C = [f 0 0; 0 f 0; 0 0 f];
            % f is the radius of the ellipse, the initialized ellipse is a circle
            % C =  [f  0  0]	
            %      [0  f  0]
            %      [0  0  f]
            
            % h = (R^T)*x, (1 0 0)^T = R^T*(p2_ - p1_)  => (p2_ - p1_) = R*(1 0 0)^T
            Ri = rotationMatrix([1 0 0]', (obj.p2_ - obj.p1_)');
            C = Ri * C * (Ri');
            axes = [f f f];
            obj.R_ = Ri;

            E = Ellipsoid(C, (obj.p1_ + obj.p2_)/2 ); 

            obs = E.points_inside(obj.obs_);
            obs_inside = obs;

            % decide short axes-1
            while (~isempty(obs_inside))
                cp = E.closest_point(obs_inside);
                rcp = (Ri')*(cp - E.d_)';   

                % Generate a new ellipse
                if(abs(rcp(1)) < axes(1)) 
                    newy = sqrt((rcp(2))^2 + (rcp(3))^2);
                    axes(2) = newy / sqrt(1 - (rcp(1)/axes(1))^2);
                end
                
                new_C = [axes(1) 0 0; 0 axes(2) 0; 0 0 axes(2)];
                E.setC(Ri * new_C * (Ri'));
                
                % Delete all the points outside the new ellipse
                obs_new = [];
                [len, ~] = size(obs_inside);
                for i = 1 : len
                % constexpr decimal_t epsilon_ = 1e-10; 
                    if(1 - E.dist(obs_inside(i,:)) > obj.epsilon_ )
                        obs_new = [obs_new; obs_inside(i,:)];
                    end
                end
                obs_inside = obs_new;
            end

            % decide short axes-2
            C = [axes(1) 0 0; 0 axes(2) 0; 0 0 axes(3)];
            E.setC(Ri * C * (Ri'));
            
            obs_inside = E.points_inside(obs);
            while (~isempty(obs_inside))
                cp = E.closest_point(obs_inside);
                rcp = (Ri')*(cp - E.d_)';
                dd = sqrt( 1 - (rcp(1)/axes(1))^2 - (rcp(2)/axes(2))^2 );
                if(dd > obj.epsilon_)
                    axes(3) = abs(rcp(3)) / dd;
                end
                
                new_C = [axes(1) 0 0; 0 axes(2) 0; 0 0 axes(3)];
                E.setC(Ri * new_C * (Ri'));
                
                % Delete all the points outside the new ellipse
                obs_new = [];
                [len, ~] = size(obs_inside);
                for i = 1 : len
                % constexpr decimal_t epsilon_ = 1e-10; 
                    if(1 - E.dist(obs_inside(i,:)) > obj.epsilon_ )
                        obs_new = [obs_new; obs_inside(i,:)];
                    end
                end
                obs_inside = obs_new;
            end
            
            E.axes_ = axes;
            obj.ellipsoid_ = E;
            
        end

        function find_polyhedron(obj)
            Vs = Polyhedron_();
            obs_remain = obj.obs_;
            while(length(obs_remain))
                plane = obj.ellipsoid_.closest_hyperplane(obs_remain);
                Vs.add(plane);
                obs_tmp = [];
                [len, ~] = size(obs_remain);
                for i = 1 : len
                    p = obs_remain(i,:);
                    if(plane.signed_dist(p) < 0)
                        obs_tmp = [obs_tmp; p];
                    end
                end
                obs_remain = obs_tmp;
            end
            obj.polyhedron_ = Vs;
        end
    end
end
