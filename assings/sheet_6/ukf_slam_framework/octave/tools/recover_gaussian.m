function [mu, sigma] = recover_gaussian(tran_points, wm)
  sines = sin(tran_points(3, :)) * transpose(wm);
  cosines = cos(tran_points(3, :)) * transpose(wm);
  theta = normalize_angle(atan2(sines, cosines));
  mu = tran_points * transpose(wm);
  mu(3) = normalize_angle(theta);
  norm = tran_points - mu;
  sigma = zeros(size(mu, 1));
  for i = 1:size(sigma, 2)
    sigma = sigma + norm(:, i) * wm(i) * norm(:, i)';
  endfor
end