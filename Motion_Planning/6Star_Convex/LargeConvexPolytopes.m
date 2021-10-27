function [A, b] = LargeConvexPolytopes(pointclouds_xyz, pos, R)
    % Given a set of pointclouds coordinates n*3 in 3d, and the current position.
    % With the user defined radius R as well.
    % Return a convex region denoted by Ax <= b
    flipping_xyz = zeros(size(pointclouds_xyz));
    for i = 1: size(pointclouds_xyz, 2)
        flipping_xyz(i, :) = SphereFlipping(pos, pointclouds_xyz(i, :), R);
    end
    k = convhull(flipping_xyz);
    sc = StarConvex(pointclouds_xyz(k, 1), pointclouds_xyz(k, 2), pos, pointclouds_xyz(k, 3));
    sc.ConstructConvexFromStar();

end