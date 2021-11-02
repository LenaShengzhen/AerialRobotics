function [A, b] = LargeConvexPolytopes(pointclouds_xyz, pos, R)
    % Given a set of pointclouds coordinates n*3 in 3d, and the current position.
    % With the user defined radius R as well.
    % Return a convex region denoted by Ax <= b
    dim = length(pos);
    assert(dim == 2 || dim == 3, "Wrong dimension! Check input data!");

    pointclouds_xyz = AddSurroundingPoints(pointclouds_xyz, pos, R);
    flipping_xyz = zeros(size(pointclouds_xyz));
    for i = 1: size(pointclouds_xyz, 1)
        flipping_xyz(i, :) = SphereFlipping(pos, pointclouds_xyz(i, :), R);
    end
    k = convhull(flipping_xyz);
    if dim == 2
        sc = StarConvex(pointclouds_xyz(k, 1), pointclouds_xyz(k, 2), pos);
    else
        sc = StarConvex(pointclouds_xyz(k, 1), pointclouds_xyz(k, 2),...
            pos, pointclouds_xyz(k, 3));
    end
    sc.ConstructConvexFromStar();
    sc.ShrinkToConvex();
    A = sc.A;
    b = sc.b;
end

function p_xyz = AddSurroundingPoints(pointclouds_xyz, pos, R)
    % If there are not enough surrounding points, add surrounding points
    % manually. For 2d, add them as a square, for 3d, add them as a cube
    dim = length(pos);
    if dim == 2
        % Add four points as a square.
        rc = R / sqrt(2);
        pointclouds_xyz(end+1, :) = [pos(1)+rc, pos(2)+rc];
        pointclouds_xyz(end+1, :) = [pos(1)-rc, pos(2)+rc];
        pointclouds_xyz(end+1, :) = [pos(1)-rc, pos(2)-rc];
        pointclouds_xyz(end+1, :) = [pos(1)+rc, pos(2)-rc];
    else
        % Add eight points as a cube.
        rc = R / sqrt(3);
        pointclouds_xyz(end+1, :) = [pos(1)+rc, pos(2)+rc, pos(3)-rc];
        pointclouds_xyz(end+1, :) = [pos(1)+rc, pos(2)-rc, pos(3)+rc];
        pointclouds_xyz(end+1, :) = [pos(1)-rc, pos(2)+rc, pos(3)+rc];
        pointclouds_xyz(end+1, :) = [pos(1)+rc, pos(2)-rc, pos(3)-rc];
        pointclouds_xyz(end+1, :) = [pos(1)-rc, pos(2)+rc, pos(3)-rc];
        pointclouds_xyz(end+1, :) = [pos(1)-rc, pos(2)-rc, pos(3)+rc];
        pointclouds_xyz(end+1, :) = [pos(1)-rc, pos(2)-rc, pos(3)-rc];
        pointclouds_xyz(end+1, :) = [pos(1)+rc, pos(2)+rc, pos(3)+rc];
    end
    p_xyz = pointclouds_xyz;
end

function p2 = SphereFlipping(pos, p1, R)
    % Sphere flipping p1 from pos by a radius R to p2
    p2 = pos + (p1 - pos) * (2 * R - norm(p1 - pos)) / norm(p1 - pos);
end
