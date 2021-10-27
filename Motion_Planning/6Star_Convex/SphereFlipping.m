function p2 = SphereFlipping(pos, p1, R)
    % Sphere flipping p1 from pos by a radius R to p2
    p2 = pos + (p1 - pos) * (2 * R - norm(p1 - pos)) / norm(p1 - pos);
end