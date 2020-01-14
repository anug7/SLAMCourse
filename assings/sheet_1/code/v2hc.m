function m = v2t(v)
  % Vector to Homogeneous coordinate representation
  % v = [x; y; theta]
  % m = [c, -s, x; s, c, y; 0, 0, 1]
  m = zeros(3);
  m(:, 3) = [v(1); v(2); 1];
  m(1:2, 1:2) = [cos(v(3)), -sin(v(3));sin(v(3)), cos(v(3))];
end