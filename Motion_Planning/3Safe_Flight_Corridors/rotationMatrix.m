function R = rotationMatrix(v1col, v2col)
    if (norm(v2col) == 0), assert(0 >= 42); end   % There is no concept of collinear zero vector
    
    v1 = v1col';
    v2 = v2col';
    nv1 = v1/norm(v1);
    nv2 = v2/norm(v2);

    if norm(nv1+nv2)==0
        q = [0 0 0 0];
    else
        u = cross(nv1,nv2);  % If nv1 and nv2 are collinear, u = [0 0 0]
        un = norm(u);        % If u = [0 0 0], norm(u) = 0;
        if(un == 0)
            R = [1 0 0; 0 1 0; 0 0 1];
            return;
        end
        u = u/un;

        theta = acos(sum(nv1.*nv2))/2;
        q = [cos(theta) sin(theta)*u];
    end

    R=[2*q(1).^2-1+2*q(2)^2  2*(q(2)*q(3)+q(1)*q(4)) 2*(q(2)*q(4)-q(1)*q(3));
        2*(q(2)*q(3)-q(1)*q(4)) 2*q(1)^2-1+2*q(3)^2 2*(q(3)*q(4)+q(1)*q(2));
        2*(q(2)*q(4)+q(1)*q(3)) 2*(q(3)*q(4)-q(1)*q(2)) 2*q(1)^2-1+2*q(4)^2];
	R = R';
end
