function v = t2v(m)
  % Homogeneous coordinate representation to Vector pose
  % m = [c, -s, x; s, c, y; 0, 0, 1]
  % v = [x; y; theta]
  v = m(:, 3);
  theta = acos(m(1, 1));
  v(3) = theta;
end